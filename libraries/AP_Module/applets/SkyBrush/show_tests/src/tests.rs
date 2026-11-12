// SPDX-License-Identifier: GPL-3.0-or-later
//! Compares the Rust port with C libskybrush (fixtures/*.oracle.txt from
//! oracle/sb_oracle.c) and the tester's Python player (*.player.txt).

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::println;
use std::string::{String, ToString};
use std::vec::Vec;

use crate::show::binary::{crc32_update, ShowFile};
use crate::show::builder::TrajectoryBuilder;
use crate::show::controller::{Controller, Scene, Screenplay};
use crate::show::events::{EventList, EventPlayer, EVENT_TYPE_PYRO};
use crate::show::fmath;
use crate::show::lights::{LightPlayer, LightProgram, Rgb, RgbwConversion};
use crate::show::poly::Poly;
use crate::show::rth_plan::{trajectory_from_entry, RthPlan, RthPlanEntry, ENTRY_HAS_TARGET_XY, ENTRY_HAS_TARGET_Z};
use crate::show::stats::{Stats, StatsCalculator};
use crate::show::time_axis::{TimeAxis, TimeSegment};
use crate::show::trajectory::{Player, Trajectory};
use crate::show::yaw::{YawControl, YawPlayer};
use crate::show::{Error, Vec3};

fn fixtures() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("fixtures")
}

fn skyb_files() -> Vec<PathBuf> {
    let mut v: Vec<_> = fs::read_dir(fixtures())
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.extension().is_some_and(|e| e == "skyb"))
        .collect();
    v.sort();
    assert!(v.len() > 40, "fixtures missing");
    v
}

/// C libskybrush output for a fixture; None if the oracle has not been generated
/// (oracle/build_oracle.sh needs the libskybrush source, which is not in this tree).
fn oracle(skyb: &Path) -> Option<Vec<Vec<String>>> {
    let text = fs::read_to_string(skyb.with_extension("oracle.txt")).ok()?;
    Some(text.lines().map(|l| l.split_whitespace().map(|s| s.to_string()).collect()).collect())
}

fn skipped(test: &str) {
    std::eprintln!("{test}: fixture outputs not generated, skipping (see fixtures/gen_fixtures.py and oracle/)");
}

fn f(s: &str) -> f32 {
    match s {
        "inf" => f32::INFINITY,
        "-inf" => f32::NEG_INFINITY,
        "nan" | "-nan" => f32::NAN,
        _ => s.parse().unwrap(),
    }
}

fn err_code<T>(r: &Result<T, Error>) -> i32 {
    match r {
        Ok(_) => 0,
        Err(e) => *e as i32,
    }
}

/// Tracks the largest deviation per category.
#[derive(Default)]
struct MaxErr(BTreeMap<&'static str, (f64, String)>);

impl MaxErr {
    fn add(&mut self, cat: &'static str, got: f32, want: f32, ctx: &str) {
        let e = if got.is_nan() && want.is_nan() || got == want {
            0.0
        } else {
            ((got as f64) - (want as f64)).abs()
        };
        let e = if e.is_nan() { f64::INFINITY } else { e };
        let entry = self.0.entry(cat).or_insert((0.0, String::new()));
        if e > entry.0 {
            *entry = (e, std::format!("{ctx}: got {got} want {want}"));
        }
    }

    fn vec(&mut self, cat: &'static str, got: Vec3, want: &[String], ctx: &str) {
        self.add(cat, got.x, f(&want[0]), ctx);
        self.add(cat, got.y, f(&want[1]), ctx);
        self.add(cat, got.z, f(&want[2]), ctx);
    }

    fn check(&self, limits: &[(&str, f64)]) {
        for (cat, (e, ctx)) in &self.0 {
            println!("max error {cat:>12}: {e:.3e}  ({ctx})");
        }
        for (cat, lim) in limits {
            let e = self.0.get(cat).map_or(0.0, |v| v.0);
            assert!(e <= *lim, "{cat}: max error {e} > {lim} ({})", self.0[cat].1);
        }
    }
}

#[test]
fn crc32_matches_ardupilot() {
    assert_eq!(crc32_update(0, b"123456789"), 0x2dfd2d88);
}

#[test]
fn container_parsing_matches_c() {
    for path in skyb_files() {
        let data = fs::read(&path).unwrap();
        let Some(o) = oracle(&path) else { continue };
        let file = ShowFile::parse(&data);
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        assert_eq!(o[0][0], "H");
        assert_eq!(err_code(&file), o[0][1].parse::<i32>().unwrap(), "{name}");
        let Ok(file) = file else { continue };
        let want: Vec<(u8, usize, usize)> = o
            .iter()
            .filter(|l| l[0] == "B")
            .map(|l| (l[1].parse().unwrap(), l[2].parse().unwrap(), l[3].parse().unwrap()))
            .collect();
        let got: Vec<(u8, usize, usize)> =
            file.blocks().map(|b| b.unwrap()).map(|b| (b.kind, b.body.len(), b.offset)).collect();
        assert_eq!(got, want, "{name}");
    }
    // truncated block and corrupted CRC
    let mut data = fs::read(fixtures().join("hdr_nocrc.skyb")).unwrap();
    data.truncate(data.len() - 1);
    let file = ShowFile::parse(&data).unwrap();
    assert!(file.blocks().any(|b| b == Err(Error::Read)));
    let mut data = fs::read(fixtures().join("first_d00.skyb")).unwrap();
    let n = data.len();
    data[n - 1] ^= 1;
    assert_eq!(ShowFile::parse(&data).err(), Some(Error::Corrupted));
    assert_eq!(ShowFile::parse(b"skyc\x02\x00").err(), Some(Error::Parse));
    assert_eq!(ShowFile::parse(b"skyb\x03\x00").err(), Some(Error::Parse));
}

fn sample_lines<'a>(o: &'a [Vec<String>], tags: &[&str]) -> impl Iterator<Item = &'a Vec<String>> {
    let tags: Vec<String> = tags.iter().map(|s| s.to_string()).collect();
    o.iter().filter(move |l| tags.contains(&l[0]))
}

fn compare_samples(err: &mut MaxErr, traj: &Trajectory, o: &[Vec<String>], tags: &[&str], ctx: &str) -> usize {
    let mut player = Player::new(traj);
    let mut n = 0;
    for l in sample_lines(o, tags) {
        let t = f(&l[1]);
        assert_eq!(l[2], "0");
        let c = std::format!("{ctx} t={t}");
        err.vec("position", player.position_at(traj, t), &l[3..6], &c);
        err.vec("velocity", player.velocity_at(traj, t), &l[6..9], &c);
        err.vec("acceleration", player.acceleration_at(traj, t), &l[9..12], &c);
        n += 1;
    }
    n
}

fn stats_fields(s: &Stats) -> Vec<f32> {
    let mut v = std::vec![
        s.duration_msec as f32,
        s.duration_sec,
        s.earliest_above_sec,
        s.takeoff_time_sec,
        s.landing_time_sec,
        s.start_to_end_distance_xy
    ];
    for p in [s.initial_pos, s.pos_at_landing_time, s.vel_at_landing_time] {
        v.extend([p.x, p.y, p.z]);
    }
    v
}

const STATS_CFG: [[f32; 4]; 4] =
    [[2500.0, 2500.0, 2500.0, 1000.0], [1000.0, 1000.0, 2000.0, 4000.0], [5000.0, 0.0, 1500.0, 500.0], [
        0.0, 3000.0, 2500.0, 1000.0,
    ]];

fn compare_stats(err: &mut MaxErr, got: &Result<Stats, Error>, l: &[String], ctx: &str) {
    assert_eq!(err_code(got).to_string(), l[1], "{ctx}");
    let Ok(s) = got else { return };
    assert_eq!(s.valid_components.to_string(), l[2], "{ctx}");
    assert_eq!(s.duration_msec.to_string(), l[3], "{ctx}");
    let want: Vec<f32> = l[3..].iter().map(|v| f(v)).collect();
    for (i, (g, w)) in stats_fields(s).iter().zip(want).enumerate() {
        let cat = match i {
            2 | 3 => "takeoff_time",
            4 => "landing_time",
            _ => "stats",
        };
        err.add(cat, *g, w, &std::format!("{ctx} field {i}"));
    }
}

fn parse_stats(l: &[String]) -> Stats {
    let v: Vec<f32> = l[4..].iter().map(|x| f(x)).collect();
    let vec = |i: usize| Vec3::new(v[i], v[i + 1], v[i + 2]);
    Stats {
        valid_components: l[2].parse().unwrap(),
        duration_msec: l[3].parse().unwrap(),
        duration_sec: v[0],
        earliest_above_sec: v[1],
        takeoff_time_sec: v[2],
        landing_time_sec: v[3],
        start_to_end_distance_xy: v[4],
        initial_pos: vec(5),
        pos_at_landing_time: vec(8),
        vel_at_landing_time: vec(11),
    }
}

fn hex(b: &[u8]) -> String {
    b.iter().map(|x| std::format!("{x:02x}")).collect()
}

#[test]
fn trajectories_stats_landing_cut_match_c() {
    let mut err = MaxErr::default();
    let (mut samples, mut files, mut landings, mut cuts) = (0, 0, 0, 0);
    for path in skyb_files() {
        let data = fs::read(&path).unwrap();
        let Some(o) = oracle(&path) else { continue };
        let name = path.file_stem().unwrap().to_string_lossy().to_string();
        let Ok(file) = ShowFile::parse(&data) else { continue };
        let tr = o.iter().find(|l| l[0] == "TR").unwrap();
        let traj = Trajectory::from_show(&file);
        assert_eq!(err_code(&traj).to_string(), tr[1], "{name}");
        let Ok(traj) = traj else { continue };
        files += 1;
        assert_eq!(traj.total_duration_msec().to_string(), tr[2], "{name}");
        assert_eq!(traj.is_empty() as u8 as i32, tr[3].parse::<i32>().unwrap(), "{name}");
        samples += compare_samples(&mut err, &traj, &o, &["P", "R"], &name);

        let mut first_stats = None;
        for (cfg, l) in STATS_CFG.iter().zip(sample_lines(&o, &["S"])) {
            let mut calc = StatsCalculator::new(1000.0);
            calc.min_ascent = cfg[0];
            calc.preferred_descent = cfg[1];
            calc.takeoff_speed = cfg[2];
            calc.acceleration = cfg[3];
            let s = calc.run(&traj);
            compare_stats(&mut err, &s, l, &name);
            first_stats.get_or_insert(s.unwrap());
        }
        assert_eq!(first_stats.unwrap(), StatsCalculator::for_show(2500.0, 2500.0, 1000.0).run(&traj).unwrap());
        // landing replacement starts from C's stats so that root-finder
        // rounding differences do not hide differences in the replacement
        let stats0 = parse_stats(sample_lines(&o, &["S"]).next().unwrap());

        let offsets = [[300.0, -200.0, 1000.0, 1000.0], [0.0, 0.0, 500.0, 0.0], [-450.0, 120.0, 2000.0, 300.0]];
        let l_lines: Vec<_> = sample_lines(&o, &["L"]).collect();
        let ls_lines: Vec<_> = sample_lines(&o, &["LS"]).collect();
        for (i, off) in offsets.iter().enumerate() {
            let mut t = traj.clone();
            let mut stats = stats0;
            let end = Vec3::new(stats.initial_pos.x + off[0], stats.initial_pos.y + off[1], stats.initial_pos.z + 1250.0);
            let r = t.replace_end_to_land_at(&mut stats, end, off[2], off[3]);
            let ctx = std::format!("{name} landing {i}");
            assert_eq!(err_code(&r).to_string(), l_lines[i][1], "{ctx}");
            assert_eq!(hex(t.bytes()), *l_lines[i].get(3).unwrap_or(&String::new()), "{ctx}");
            compare_stats(&mut err, &r.map(|_| stats), ls_lines[i], &ctx);
            if r.is_ok() {
                landings += 1;
                if i == 0 {
                    samples += compare_samples(&mut err, &t, &o, &["LP", "LR"], &ctx);
                }
            }
        }

        for l in sample_lines(&o, &["C"]) {
            let mut t = traj.clone();
            let r = t.cut_at(f(&l[1]));
            assert_eq!(err_code(&r).to_string(), l[2], "{name} cut {}", l[1]);
            assert_eq!(hex(t.bytes()), *l.get(3).unwrap_or(&String::new()), "{name} cut {}", l[1]);
            cuts += 1;
        }
    }
    println!("{files} trajectories, {samples} samples, {landings} landing replacements, {cuts} cuts");
    err.check(&[
        ("position", 1e-3),
        ("velocity", 1e-3),
        ("acceleration", 1e-2),
        ("takeoff_time", 1e-4),
        ("landing_time", 1e-4),
        ("stats", 1e-2),
    ]);
}

#[test]
fn touches_matches_c() {
    let Ok(text) = fs::read_to_string(fixtures().join("touches.oracle.txt")) else {
        return skipped("touches_matches_c");
    };
    let (mut n, mut hits, mut mismatch, mut better, mut worse) = (0, 0, 0, 0, 0);
    let mut err = MaxErr::default();
    for line in text.lines() {
        let v: Vec<&str> = line.split_whitespace().collect();
        let k: usize = v[1].parse().unwrap();
        let coeffs: Vec<f32> = v[2..2 + k].iter().map(|s| f(s)).collect();
        let value = f(v[2 + k]);
        let hit = v[3 + k] == "1";
        let want = f(v[4 + k]);
        let got = Poly::new(&coeffs).touches(value);
        n += 1;
        match got {
            Some(r) if hit => {
                hits += 1;
                err.add("touch_root", r, want, line);
                let res = |t: f32| {
                    let t = t as f64;
                    (coeffs.iter().rev().fold(0.0f64, |a, &c| a * t + c as f64) - value as f64).abs()
                };
                if res(r) < res(want) {
                    better += 1;
                } else if res(r) > res(want) {
                    worse += 1;
                }
            }
            None if !hit => {}
            _ => {
                mismatch += 1;
                println!("touch mismatch: {line} got {got:?}");
            }
        }
    }
    println!("{n} polynomials, {hits} touching, {mismatch} hit/miss mismatches");
    println!("root residual vs C (cpowf): {better} smaller, {worse} larger, rest equal");
    err.check(&[("touch_root", 2e-3)]);
    assert!(mismatch <= n / 1000, "too many mismatches");
}

#[test]
fn lights_match_c() {
    let (mut files, mut samples) = (0, 0);
    for path in skyb_files() {
        let data = fs::read(&path).unwrap();
        let Some(o) = oracle(&path) else { continue };
        let name = path.file_stem().unwrap().to_string_lossy().to_string();
        let Some(lp) = o.iter().find(|l| l[0] == "LIGHT") else { continue };
        let file = ShowFile::parse(&data).unwrap();
        let prog = LightProgram::from_show(&file);
        assert_eq!(err_code(&prog).to_string(), lp[1], "{name}");
        let Ok(prog) = prog else { continue };
        files += 1;
        let mut player = LightPlayer::new(&prog);
        for l in sample_lines(&o, &["K", "KR"]) {
            let t: u64 = l[1].parse().unwrap();
            let c = player.color_at(&prog, t);
            let want = Rgb::new(l[2].parse().unwrap(), l[3].parse().unwrap(), l[4].parse().unwrap());
            assert_eq!(c, want, "{name} t={t}");
            samples += 1;
        }
    }
    println!("{files} light programs, {samples} colour samples, all exact");
    if files == 0 {
        return skipped("lights_match_c");
    }
    assert!(files > 30);
}

#[test]
fn positions_match_python_player() {
    let mut worst: f64 = 0.0;
    let mut n = 0;
    for path in skyb_files() {
        let player_file = path.with_extension("player.txt");
        let Ok(text) = fs::read_to_string(&player_file) else { continue };
        let data = fs::read(&path).unwrap();
        let file = ShowFile::parse(&data).unwrap();
        let traj = Trajectory::from_show(&file).unwrap();
        let scale = traj.scale() as f64;
        let mut p = Player::new(&traj);
        let mut file_worst: f64 = 0.0;
        for line in text.lines() {
            let v: Vec<f64> = line.split_whitespace().map(|s| s.parse().unwrap()).collect();
            let pos = p.position_at(&traj, v[0] as f32);
            let d = ((pos.x as f64 - v[1]).powi(2) + (pos.y as f64 - v[2]).powi(2) + (pos.z as f64 - v[3]).powi(2)).sqrt();
            file_worst = file_worst.max(d / scale);
            n += 1;
        }
        // encoder truncates to the coordinate grid (and rounds quadratic
        // control points), so allow ~2 grid steps per axis
        assert!(file_worst < 3.5, "{}: {file_worst} grid steps", path.display());
        worst = worst.max(file_worst);
    }
    println!("{n} Python player samples, max deviation {worst:.3} coordinate steps (scale units)");
    if n == 0 {
        return skipped("positions_match_python_player");
    }
    assert!(n > 1000);
}

#[test]
fn fmath_accuracy() {
    let mut vals = std::vec![0.0f32, -0.0, 1.0, -1.0, 0.5, 2.0, 1e-30, 1e30, f32::MIN_POSITIVE, 1e-40, 3.0e38];
    let mut s = 0x1234_5678u32;
    for _ in 0..200_000 {
        s = s.wrapping_mul(1664525).wrapping_add(1013904223);
        vals.push(f32::from_bits(s));
        vals.push((s >> 8) as f32 / 16777216.0 * 20.0 - 10.0);
    }
    let ulps = |a: f32, b: f32| -> u32 {
        if a == b || (a.is_nan() && b.is_nan()) {
            0
        } else if a.is_nan() != b.is_nan() || a.is_infinite() || b.is_infinite() {
            u32::MAX
        } else {
            ((a.to_bits() as i64) - (b.to_bits() as i64)).unsigned_abs() as u32
        }
    };
    let mut worst = BTreeMap::new();
    let mut note = |name: &'static str, got: f32, want: f32, x: f32| {
        let u = ulps(got, want);
        let e = worst.entry(name).or_insert((0u32, 0.0f32));
        if u > e.0 {
            *e = (u, x);
        }
    };
    for &x in &vals {
        note("sqrtf", fmath::sqrtf(x), x.sqrt(), x);
        note("cbrtf", fmath::cbrtf(x), x.cbrt(), x);
        note("floorf", fmath::floorf(x), x.floor(), x);
        note("ceilf", fmath::ceilf(x), x.ceil(), x);
        note("roundf", fmath::roundf(x), x.round(), x);
        note("truncf", fmath::truncf(x), x.trunc(), x);
        if x.abs() < 1e6 {
            note("cosf", fmath::cosf(x), x.cos(), x);
        }
        note("acosf", fmath::acosf(x), x.acos(), x);
        note("logf", fmath::logf(x), x.ln(), x);
        if x.abs() < 100.0 {
            note("expf", fmath::expf(x), x.exp(), x);
            note("powf", fmath::powf(x.abs() + 1.0, 0.37), (x.abs() + 1.0).powf(0.37), x);
        }
        let y = f32::from_bits(x.to_bits().rotate_left(7));
        note("hypotf", fmath::hypotf(x, y), x.hypot(y), x);
        let xd = x as f64 * 1.000000123456789;
        let r = fmath::sqrt(xd);
        assert!(r == xd.sqrt() || (r.is_nan() && xd.sqrt().is_nan()), "sqrt({xd})");
    }
    for (name, (u, x)) in &worst {
        println!("fmath {name:>7}: max {u} ulp (at {x:e})");
    }
    let limit = |n: &str| match n {
        "sqrtf" | "floorf" | "ceilf" | "roundf" | "truncf" | "cbrtf" | "hypotf" => 1,
        _ => 2,
    };
    for (name, (u, x)) in &worst {
        assert!(*u <= limit(name), "{name}: {u} ulp at {x:e}");
    }
}

#[test]
fn controller_single_scene() {
    let data = fs::read(fixtures().join("circle-7m-radius_d00.skyb")).unwrap();
    let mut ctrl = Controller::from_show_bytes(&data).unwrap();
    let file = ShowFile::parse(&data).unwrap();
    let traj = Trajectory::from_show(&file).unwrap();
    let prog = LightProgram::from_show(&file).unwrap();
    let stats = StatsCalculator::for_show(2500.0, 2500.0, 1000.0).run(&traj).unwrap();
    let end_ms = (stats.landing_time_sec * 1000.0) as u32;
    ctrl.screenplay_mut().scenes[0].duration_msec = end_ms;
    ctrl.notify_screenplay_changed();
    let mut p = Player::new(&traj);
    let mut lp = LightPlayer::new(&prog);
    for t_ms in (0..end_ms).step_by(97) {
        ctrl.update_time_msec(t_ms).unwrap();
        let out = *ctrl.output();
        let t = t_ms as f32 / 1000.0;
        assert_eq!(out.position, Some(p.position_at(&traj, t)));
        assert_eq!(out.velocity, Some(p.velocity_at(&traj, t)));
        assert_eq!(out.color, Some(lp.color_at(&prog, t_ms as u64)));
        assert_eq!(out.yaw, None);
        assert!(!ctrl.has_reached_end());
        assert_eq!(ctrl.output_time().scene, Some(0));
    }
    ctrl.update_time_msec(end_ms).unwrap();
    assert!(ctrl.has_reached_end());
    assert_eq!(ctrl.output().position, None);
    assert_eq!(ctrl.output().velocity, Some(Vec3::ZERO));
    // going back in time works and replays
    ctrl.update_time_msec(1000).unwrap();
    assert!(!ctrl.has_reached_end());
    assert_eq!(ctrl.output().position, Some(Player::new(&traj).position_at(&traj, 1.0)));

    // landing replacement through the controller's scene
    let mut s = stats;
    let start = s.initial_pos;
    let target = Vec3::new(start.x + 200.0, start.y - 100.0, start.z + 1250.0);
    let sc = &mut ctrl.screenplay_mut().scenes[0];
    sc.trajectory.as_mut().unwrap().replace_end_to_land_at(&mut s, target, 1000.0, 1000.0).unwrap();
    sc.duration_msec = sc.time_axis.reverse_map(s.duration_sec) as u32;
    ctrl.notify_trajectory_changed();
    ctrl.update_time_msec(sc_duration(&ctrl) - 1).unwrap();
    let pos = ctrl.output().position.unwrap();
    assert!((pos.x - target.x).abs() < 2.0 && (pos.y - target.y).abs() < 2.0 && (pos.z - target.z).abs() < 5.0, "{pos:?}");
    let expect = stats.landing_time_sec + (stats.pos_at_landing_time.z - target.z) / 1000.0;
    assert!((s.landing_time_sec - expect).abs() < 1e-3);
}

fn sc_duration(c: &Controller) -> u32 {
    c.screenplay().scenes[0].duration_msec
}

#[test]
fn landing_replacement_on_fixtures() {
    // the tester's landing-accuracy check: end exactly above the takeoff point
    let mut n = 0;
    for path in skyb_files().iter().filter(|p| !p.to_string_lossy().contains("synth") && !p.to_string_lossy().contains("hdr")) {
        let data = fs::read(path).unwrap();
        let traj0 = Trajectory::from_show(&ShowFile::parse(&data).unwrap()).unwrap();
        let mut stats = StatsCalculator::for_show(2500.0, 2500.0, 1000.0).run(&traj0).unwrap();
        if stats.start_to_end_distance_xy > 500.0 {
            continue;
        }
        let mut traj = traj0.clone();
        let target = Vec3::new(stats.initial_pos.x - 321.0, stats.initial_pos.y + 123.0, stats.initial_pos.z + 1250.0);
        let before = stats;
        traj.replace_end_to_land_at(&mut stats, target, 1000.0, 1000.0).unwrap();
        let end = traj.end_position();
        let scale = traj.scale();
        assert!((end.x - target.x).abs() <= scale && (end.y - target.y).abs() <= scale && (end.z - target.z).abs() <= scale);
        let expect = before.landing_time_sec + (before.pos_at_landing_time.z - target.z) / 1000.0;
        assert!((stats.landing_time_sec - expect).abs() < 2e-3, "{}", path.display());
        assert!(stats.duration_msec.abs_diff(traj.total_duration_msec()) <= 1);
        n += 1;
    }
    println!("{n} circular fixture trajectories corrected");
    assert!(n >= 5);
}

#[test]
fn builder_move_and_hold() {
    let mut b = TrajectoryBuilder::new(1).unwrap();
    b.set_start_position(Vec3::new(0.0, 0.0, 0.0)).unwrap();
    b.hold_position_for(130_000).unwrap();
    b.move_to_in_time(Vec3::new(10000.0, -5000.0, 3000.0), 10_000, 2000.0).unwrap();
    b.append_line(Vec3::new(0.0, 0.0, 0.0), 70_000).unwrap();
    assert_eq!(b.append_line(Vec3::new(1e9, 0.0, 0.0), 1000), Err(Error::Inval));
    assert_eq!(b.last_position(), Vec3::ZERO);
    let traj = b.into_trajectory().unwrap();
    assert_eq!(traj.total_duration_msec(), 210_000);
    let mut p = Player::new(&traj);
    let at = |p: &mut Player, t: f32| p.position_at(&traj, t);
    let e = at(&mut p, 140.0);
    assert!((e.x - 10000.0).abs() < 0.01 && (e.y + 5000.0).abs() < 0.01 && (e.z - 3000.0).abs() < 0.01, "{e:?}");
    let v = p.velocity_at(&traj, 140.0);
    assert!(v.x.abs() < 0.01 && v.y.abs() < 0.01 && v.z.abs() < 0.01, "{v:?}");
    let m = at(&mut p, 175.0);
    assert!((m.x - 5000.0).abs() < 1.0 && (m.y + 2500.0).abs() < 1.0 && (m.z - 1500.0).abs() < 1.0, "{m:?}");
    assert_eq!(at(&mut p, 1000.0), Vec3::ZERO);
    let v = p.velocity_at(&traj, 135.0);
    assert!(v.x > 1000.0 && v.y < -500.0, "{v:?}");
}

#[test]
fn colors_and_constant_program() {
    for c in [Rgb::new(255, 0, 0), Rgb::new(0, 255, 0), Rgb::new(0, 0, 255), Rgb::new(255, 255, 255)] {
        assert_eq!(Rgb::decode_rgb565(c.encode_rgb565()).encode_rgb565(), c.encode_rgb565());
    }
    assert_eq!(Rgb::new(255, 255, 255).encode_rgb565(), 0xffff);
    assert_eq!(Rgb::decode_rgb565(0xf800), Rgb::new(0xf8, 0, 0));
    let prog = LightProgram::constant_color(Rgb::new(1, 2, 3));
    let mut p = LightPlayer::new(&prog);
    assert_eq!(p.color_at(&prog, 0), Rgb::new(1, 2, 3));
    assert_eq!(p.color_at(&prog, 1_000_000), Rgb::new(1, 2, 3));
    // fade from black to white over 1 s, then loop twice red/black 100 ms
    let prog = LightProgram::from_bytes(&[0x0B, 50, 0x0C, 2, 0x04, 255, 0, 0, 5, 0x06, 5, 0x0D, 0x00]);
    let mut p = LightPlayer::new(&prog);
    assert_eq!(p.color_at(&prog, 500), Rgb::new(127, 127, 127));
    assert_eq!(p.color_at(&prog, 1050), Rgb::new(255, 0, 0));
    assert_eq!(p.color_at(&prog, 1150), Rgb::BLACK);
    assert_eq!(p.color_at(&prog, 1250), Rgb::new(255, 0, 0));
    assert!(p.seek(&prog, 2000));
    let w = Rgb::new(200, 150, 100).to_rgbw(&RgbwConversion::SubtractMin);
    assert_eq!((w.r, w.g, w.b, w.w), (100, 50, 0, 100));
    let warm = Rgb::from_color_temperature(2700.0);
    assert_eq!(warm.r, 255);
    assert!(warm.g > 150 && warm.g < 180 && warm.b > 80 && warm.b < 120, "{warm:?}");
    let conv = RgbwConversion::color_temperature(4000.0);
    let w = Rgb::WHITE.to_rgbw(&conv);
    assert!(w.w > 200, "{w:?}");
}

#[test]
fn events_yaw_time_axis() {
    let mut raw = Vec::new();
    for (t, kind, sub, payload) in [(1000u32, EVENT_TYPE_PYRO, 3u8, 0u32), (2000, 2, 0, 7), (2000, EVENT_TYPE_PYRO, 3, u32::MAX)] {
        raw.extend(t.to_le_bytes());
        raw.extend([kind, sub]);
        raw.extend(payload.to_le_bytes());
    }
    raw.push(0xAA); // trailing partial entry is ignored
    let mut ev = EventList::from_bytes(&raw).unwrap();
    assert_eq!(ev.len(), 3);
    ev.adjust_timestamps_by_type(EVENT_TYPE_PYRO, -1500);
    assert_eq!(ev.events.iter().map(|e| e.time_msec).collect::<Vec<_>>(), std::vec![0, 500, 2000]);
    let mut p = EventPlayer::default();
    assert_eq!(p.next_event_not_later_than(&ev, 0.4).map(|e| e.time_msec), Some(0));
    assert!(p.next_event_not_later_than(&ev, 0.4).is_none());
    assert!(p.next_event_not_later_than(&ev, 0.5).unwrap().is_pyro_off());
    let mut bad = raw.clone();
    bad[0..4].copy_from_slice(&5000u32.to_le_bytes());
    assert_eq!(EventList::from_bytes(&bad).err(), Some(Error::Corrupted));

    // yaw: offset 90 deg, +45 deg over 2 s, then -90 deg over 1 s
    let yaw = YawControl::from_bytes(&[1, 0x84, 0x03, 0xd0, 0x07, 0xc2, 0x01, 0xe8, 0x03, 0x7c, 0xfc]).unwrap();
    assert!(yaw.auto_yaw);
    let mut yp = YawPlayer::new(&yaw);
    assert_eq!(yp.yaw_at(&yaw, 1.0), 112.5);
    assert_eq!(yp.yaw_rate_at(&yaw, 1.0), 22.5);
    assert_eq!(yp.yaw_at(&yaw, 2.5), 90.0);
    assert_eq!(yp.yaw_at(&yaw, 10.0), 45.0);
    assert_eq!(yp.yaw_rate_at(&yaw, 10.0), 0.0);
    assert_eq!(YawControl::from_bytes(&[0, 0]).err(), Some(Error::Corrupted));

    // time axis: realtime 10 s, slow down to 0 over 4 s, frozen 2 s, back to 1 over 2 s
    let mut ax = TimeAxis::default();
    ax.set_origin_msec(1000).unwrap();
    assert_eq!(ax.map(500), (-0.5, 1.0));
    ax.push(TimeSegment::realtime(10_000)).unwrap();
    ax.push(TimeSegment::new(4000, 1.0, 0.0)).unwrap();
    ax.push(TimeSegment::new(2000, 0.0, 0.0)).unwrap();
    ax.push(TimeSegment::new(2000, 0.0, 1.0)).unwrap();
    assert_eq!(ax.map(6000), (5.0, 1.0));
    assert_eq!(ax.map(13000), (11.5, 0.5));
    assert_eq!(ax.map(16000).0, 12.0);
    assert_eq!(ax.map(19000), (13.0, 1.0));
    assert_eq!(ax.map(21000), (15.0, 1.0));
    for w in [0.0f32, 3.0, 10.0, 11.5, 12.5, 13.0, 20.0] {
        let wall = ax.reverse_map(w);
        assert!((ax.map(wall).0 - w).abs() < 2e-3, "w={w} wall={wall}");
    }
    assert_eq!(TimeSegment::warped(2.0, 1.0, 0.0).unwrap().duration_msec, 4000);

    // a two-scene screenplay
    let data = fs::read(fixtures().join("first_d00.skyb")).unwrap();
    let mut sp = Screenplay::from_show_bytes(&data).unwrap();
    sp.scenes[0].duration_msec = 5000;
    let mut s2 = Scene { lights: Some(LightProgram::constant_color(Rgb::new(9, 9, 9))), ..Default::default() };
    s2.time_axis.set_origin_msec(5000).unwrap();
    sp.scenes.push(s2);
    let mut c = Controller::new(sp);
    c.update_time_msec(4999).unwrap();
    assert_eq!(c.current_scene(), Some(0));
    c.update_time_msec(6000).unwrap();
    assert_eq!(c.current_scene(), Some(1));
    assert_eq!(c.output().color, Some(Rgb::new(9, 9, 9)));
    assert_eq!(c.output().position, None);
    assert_eq!(c.output_time().time_in_scene_msec, 1000);
    assert!(!c.has_reached_end());
}

fn entry_fields(e: &RthPlanEntry) -> [f32; 13] {
    [
        e.time_sec,
        e.duration_sec,
        e.flags as f32,
        e.landing_target.0,
        e.landing_target.1,
        e.arrival_altitude,
        e.pre_delay_sec,
        e.post_delay_sec,
        e.pre_neck,
        e.pre_neck_duration_sec,
        e.max_acceleration,
        e.landing_velocity,
        e.landing_altitude,
    ]
}

fn parse_entry(l: &[String]) -> RthPlanEntry {
    let v: Vec<f32> = l[..13].iter().map(|s| f(s)).collect();
    RthPlanEntry {
        time_sec: v[0],
        duration_sec: v[1],
        flags: v[2] as u8,
        landing_target: (v[3], v[4]),
        arrival_altitude: v[5],
        pre_delay_sec: v[6],
        post_delay_sec: v[7],
        pre_neck: v[8],
        pre_neck_duration_sec: v[9],
        max_acceleration: v[10],
        landing_velocity: v[11],
        landing_altitude: v[12],
    }
}

fn same(a: f32, b: f32) -> bool {
    a == b || a.is_nan() && b.is_nan()
}

#[test]
fn rth_plan_matches_c() {
    let mut files = 0;
    let (mut n_eval, mut n_traj) = (0, 0);
    for path in skyb_files().iter().filter(|p| p.to_string_lossy().contains("rth_")) {
        let data = fs::read(path).unwrap();
        let Some(o) = oracle(path) else { continue };
        let name = path.file_name().unwrap().to_string_lossy().to_string();
        let file = ShowFile::parse(&data).unwrap();
        let plan = RthPlan::from_show(&file);
        let rp = o.iter().find(|l| l[0] == "RP").unwrap();
        assert_eq!(err_code(&plan), rp[1].parse::<i32>().unwrap(), "{name}");
        // the screenplay fails to load with a corrupted RTH plan, as in C
        assert_eq!(Screenplay::from_show_bytes(&data).is_ok(), plan.is_ok(), "{name}");
        files += 1;
        let Ok(plan) = plan else { continue };
        assert_eq!(plan.num_points(), rp[2].parse::<usize>().unwrap(), "{name}");
        assert_eq!(plan.num_entries(), rp[3].parse::<usize>().unwrap(), "{name}");
        assert!(same(plan.default_acceleration_limit(), f(&rp[4])), "{name}");
        assert!(same(plan.default_landing_velocity(), f(&rp[5])), "{name}");
        assert!(same(plan.default_landing_altitude(), f(&rp[6])), "{name}");
        for (i, l) in o.iter().filter(|l| l[0] == "RPT").enumerate() {
            assert_eq!(plan.point(i).unwrap(), (f(&l[1]), f(&l[2])), "{name} point {i}");
        }
        for l in o.iter() {
            match l[0].as_str() {
                "RE" => {
                    let t = f(&l[1]);
                    let r = plan.evaluate_at(t);
                    assert_eq!(err_code(&r), l[2].parse::<i32>().unwrap(), "{name} t={t}");
                    if let Ok(e) = r {
                        let want = parse_entry(&l[3..]);
                        let (g, w) = (entry_fields(&e), entry_fields(&want));
                        assert!(g.iter().zip(w.iter()).all(|(a, b)| same(*a, *b)), "{name} t={t}: {e:?} != {want:?}");
                        n_eval += 1;
                    }
                }
                "RT" | "RT0" | "RTC" => {
                    let start = Vec3::new(f(&l[2]), f(&l[3]), f(&l[4]));
                    let e = parse_entry(&l[5..]);
                    let r = trajectory_from_entry(&e, start);
                    assert_eq!(err_code(&r), l[1].parse::<i32>().unwrap(), "{name} {}: {e:?}", l[0]);
                    if let Ok(traj) = r {
                        assert_eq!(hex(traj.bytes()), l[18], "{name} {}: {e:?} from {start:?}", l[0]);
                        n_traj += 1;
                    }
                }
                _ => {}
            }
        }
    }
    if files == 0 {
        return skipped("rth_plan_matches_c");
    }
    println!("{files} RTH fixtures, {n_eval} evaluations, {n_traj} trajectories identical to C");
    assert!(n_eval > 100 && n_traj > 100);
}

#[test]
fn rth_plan_land_in_place() {
    // default entry without a plan: hold, no descent (no landing velocity)
    let e = RthPlan::default_entry(None, 2.0);
    let traj = trajectory_from_entry(&e, Vec3::new(1000.0, 2000.0, 5000.0)).unwrap();
    assert_eq!(traj.total_duration_msec(), 2000);
    assert_eq!(traj.end_position(), Vec3::new(1000.0, 2000.0, 5000.0));
    // with a target, a landing velocity and an acceleration limit
    let e = RthPlanEntry {
        flags: e.flags | ENTRY_HAS_TARGET_XY | ENTRY_HAS_TARGET_Z,
        landing_target: (-3000.0, 4000.0),
        arrival_altitude: 6000.0,
        duration_sec: 10.0,
        landing_velocity: 1000.0,
        landing_altitude: 0.0,
        max_acceleration: 2000.0,
        time_sec: 0.0,
        ..e
    };
    let traj = trajectory_from_entry(&e, Vec3::new(1000.0, 2000.0, 5000.0)).unwrap();
    assert_eq!(traj.total_duration_msec(), 16000);
    assert_eq!(traj.end_position(), Vec3::new(-3000.0, 4000.0, 0.0));
    let mut p = Player::new(&traj);
    let m = p.position_at(&traj, 10.0);
    assert!((m.x + 3000.0).abs() < 1.0 && (m.z - 6000.0).abs() < 1.0, "{m:?}");
}

#[test]
fn screenplay_scene_tags_and_replacement() {
    let data = fs::read(fixtures().join("first_d00.skyb")).unwrap();
    let mut sp = Screenplay::from_show_bytes(&data).unwrap();
    assert!(sp.rth_plan.is_none());
    sp.scenes[0].tag = 1;
    // realtime: show clock == warped time
    assert_eq!(sp.time_sec_for_tag_and_warped_time(1, 12.5), 12.5);
    assert_eq!(sp.time_sec_for_tag_and_warped_time(2, 1.0), f32::INFINITY);
    // half speed from the origin: warped 10 s is reached at 20 s
    let ax = &mut sp.scenes[0].time_axis;
    ax.push(TimeSegment::new(u32::MAX, 0.5, 0.5)).unwrap();
    assert_eq!(sp.time_sec_for_tag_and_warped_time(1, 10.0), 20.0);
    assert_eq!(sp.time_sec_for_tag_and_warped_time(1, f32::NAN), f32::INFINITY);
    // a scene limited to 15 s does not reach warped 10 s
    sp.scenes[0].duration_msec = 15000;
    assert_eq!(sp.time_sec_for_tag_and_warped_time(1, 10.0), f32::INFINITY);
    // an earlier scene covering that time hides the tagged one
    sp.scenes[0].duration_msec = u32::MAX;
    let mut first = Scene { duration_msec: 30000, ..Default::default() };
    first.tag = 2;
    sp.scenes.insert(0, first);
    assert_eq!(sp.time_sec_for_tag_and_warped_time(1, 10.0), f32::INFINITY);

    // dynamic scene coverage
    let mut s = Screenplay::from_show_bytes(&data).unwrap().scenes.remove(0);
    let total = s.trajectory.as_ref().unwrap().total_duration_sec();
    assert_eq!(s.uncovered_trajectory_duration_sec(), Ok(total));
    s.time_axis.push(TimeSegment::new(4000, 1.0, 0.0)).unwrap();
    assert_eq!(s.uncovered_trajectory_duration_sec(), Ok(total - 2.0));
    s.time_axis.push(TimeSegment::new(u32::MAX, 0.0, 0.0)).unwrap();
    assert_eq!(s.uncovered_trajectory_duration_sec(), Ok(0.0));

    // replacing the screenplay keeps the player of an unchanged current scene
    let base = Screenplay::from_show_bytes(&data).unwrap();
    let mut c = Controller::new(base.clone());
    c.update_time_msec(3000).unwrap();
    let before = *c.output();
    c.replace_screenplay(base.clone());
    assert!(!c.is_output_valid());
    c.update_time_msec(3000).unwrap();
    assert_eq!(*c.output(), before);
    let mut other = base.clone();
    let traj = TrajectoryBuilder::new(1).unwrap().into_trajectory().unwrap();
    other.scenes[0].trajectory = Some(traj);
    c.replace_screenplay(other);
    c.update_time_msec(3000).unwrap();
    assert_eq!(c.output().position, Some(Vec3::ZERO));
}
