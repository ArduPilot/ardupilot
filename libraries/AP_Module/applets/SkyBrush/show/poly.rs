// SPDX-License-Identifier: GPL-3.0-or-later
// Port of libskybrush trajectory/poly.c and bezier_cut_at (Copyright CollMot Robotics Ltd).
//! Power-basis polynomials up to degree 7 on t in [0, 1].

use super::fmath::{acosf, cbrtf, cosf, sqrtf};
use super::{Error, Result, Vec3};

pub const MAX_COEFFS: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Poly {
    pub coeffs: [f32; MAX_COEFFS],
    pub num_coeffs: u8,
}

fn is_zero(x: f32) -> bool {
    x.abs() < f32::MIN_POSITIVE
}

const FACS: [i32; MAX_COEFFS] = [1, 1, 2, 6, 24, 120, 720, 5040];

impl Default for Poly {
    fn default() -> Self {
        Poly::constant(0.0)
    }
}

impl Poly {
    pub fn new(xs: &[f32]) -> Self {
        let n = xs.len().min(MAX_COEFFS);
        let mut p = Poly { coeffs: [0.0; MAX_COEFFS], num_coeffs: n as u8 };
        p.coeffs[..n].copy_from_slice(&xs[..n]);
        p
    }

    pub fn constant(x: f32) -> Self {
        let mut p = Poly { coeffs: [0.0; MAX_COEFFS], num_coeffs: 1 };
        p.coeffs[0] = x;
        p
    }

    pub fn linear(duration: f32, x0: f32, x1: f32) -> Self {
        let mut p = Poly { coeffs: [0.0; MAX_COEFFS], num_coeffs: 2 };
        if duration.abs() >= f32::EPSILON {
            p.coeffs[0] = x0;
            p.coeffs[1] = (x1 - x0) / duration;
        } else {
            p.coeffs[0] = (x0 + x1) / 2.0;
        }
        p
    }

    /// Bezier control points to power basis, stretched to `duration`.
    pub fn bezier(duration: f32, xs: &[f32]) -> Self {
        match xs.len() {
            0 => Poly::constant(0.0),
            1 => Poly::constant(xs[0]),
            2 => Poly::linear(duration, xs[0], xs[1]),
            len => {
                let n = len.min(MAX_COEFFS) - 1;
                let mut p = Poly { coeffs: [0.0; MAX_COEFFS], num_coeffs: (n + 1) as u8 };
                for j in 0..=n {
                    let mut coeff = 0.0f32;
                    let mut sign: i32 = if j % 2 == 1 { -1 } else { 1 };
                    for i in 0..=j {
                        coeff += sign as f32 * xs[i] / FACS[i] as f32 / FACS[j - i] as f32;
                        sign = -sign;
                    }
                    p.coeffs[j] = coeff * FACS[n] as f32 / FACS[n - j] as f32;
                }
                p.stretch(duration);
                p
            }
        }
    }

    pub fn coeffs(&self) -> &[f32] {
        &self.coeffs[..self.num_coeffs as usize]
    }

    pub fn eval(&self, t: f32) -> f32 {
        let mut r = 0.0f32;
        for &c in self.coeffs().iter().rev() {
            r = r * t + c;
        }
        r
    }

    pub fn degree(&self) -> u8 {
        self.num_coeffs.saturating_sub(1)
    }

    pub fn deriv(&mut self) {
        let n = self.num_coeffs as usize;
        if n > 1 {
            for i in 1..n {
                self.coeffs[i - 1] = i as f32 * self.coeffs[i];
            }
            self.num_coeffs -= 1;
            self.coeffs[n - 1] = 0.0;
        } else {
            *self = Poly::constant(0.0);
        }
    }

    pub fn scale(&mut self, factor: f32) {
        let n = self.num_coeffs as usize;
        for c in &mut self.coeffs[..n] {
            *c *= factor;
        }
    }

    pub fn stretch(&mut self, factor: f32) {
        let factor = 1.0 / factor;
        let mut scale = factor;
        for i in 1..self.num_coeffs as usize {
            self.coeffs[i] *= scale;
            scale *= factor;
        }
    }

    fn significant_coeffs(&self) -> usize {
        let mut i = self.num_coeffs as usize;
        if i == 0 {
            return 0;
        }
        while i > 1 && is_zero(self.coeffs[i - 1]) {
            i -= 1;
        }
        i
    }

    /// Real roots of `p(t) = rhs` (degree <= 3). Returns the root count.
    pub fn solve(&self, rhs: f32, roots: &mut [f32; 3]) -> Result<usize> {
        match self.significant_coeffs() {
            0 => Ok(0),
            1 => Ok(self.solve_1d(rhs, roots)),
            2 => Ok(self.solve_2d(rhs, roots)),
            3 => Ok(self.solve_3d(rhs, roots)),
            4 => Ok(self.solve_4d(rhs, roots)),
            _ => Err(Error::Unimplemented),
        }
    }

    fn solve_1d(&self, rhs: f32, roots: &mut [f32; 3]) -> usize {
        if is_zero(self.coeffs[0] - rhs) {
            roots[0] = 0.0;
            1
        } else {
            0
        }
    }

    fn solve_2d(&self, rhs: f32, roots: &mut [f32; 3]) -> usize {
        let a = self.coeffs[1];
        let b = self.coeffs[0] - rhs;
        if is_zero(a) {
            0
        } else {
            roots[0] = -b / a;
            1
        }
    }

    fn solve_3d(&self, rhs: f32, roots: &mut [f32; 3]) -> usize {
        let a = self.coeffs[2];
        let b = self.coeffs[1];
        let c = self.coeffs[0] - rhs;
        if is_zero(a) {
            return self.solve_2d(rhs, roots);
        }
        let d = b * b - 4.0 * a * c;
        if is_zero(d) {
            roots[0] = -b / (2.0 * a);
            1
        } else if d > 0.0 {
            let d = sqrtf(d);
            roots[0] = (-b - d) / (2.0 * a);
            roots[1] = (-b + d) / (2.0 * a);
            2
        } else {
            0
        }
    }

    // Cardano as in libskybrush; the three-real-root case uses the
    // trigonometric form instead of complex cpowf (same principal roots).
    fn solve_4d(&self, rhs: f32, roots: &mut [f32; 3]) -> usize {
        let a = self.coeffs[3];
        let b = self.coeffs[2];
        let c = self.coeffs[1];
        let d = self.coeffs[0] - rhs;
        if is_zero(a) {
            return self.solve_3d(rhs, roots);
        }
        let p = (3.0 * a * c - b * b) / (3.0 * a * a);
        let q = (2.0 * b * b * b - 9.0 * a * b * c + 27.0 * a * a * d) / (27.0 * a * a * a);
        let delta = (q * q) / 4.0 + (p * p * p) / 27.0;
        let offset = -b / (3.0 * a);
        let n;
        if delta.abs() < 1e-8 {
            let u = cbrtf(-q / 2.0);
            if u == 0.0 {
                n = 1;
                roots[0] = 0.0;
            } else {
                n = 2;
                roots[0] = 2.0 * u;
                roots[1] = -u;
            }
        } else if delta > 0.0 {
            n = 1;
            roots[0] = cbrtf(-q / 2.0 + sqrtf(delta)) + cbrtf(-q / 2.0 - sqrtf(delta));
        } else {
            // u = cbrt(-q/2 + i*sqrt(-delta)), v = conj(u); |u| = sqrt(-p/3)
            let m = sqrtf(-p / 3.0);
            let r = sqrtf(q * q / 4.0 - delta);
            let phi = acosf((-q / 2.0 / r).clamp(-1.0, 1.0)) / 3.0;
            const TWO_PI_3: f32 = 2.0 * core::f32::consts::PI / 3.0;
            n = 3;
            roots[0] = 2.0 * m * cosf(phi);
            roots[1] = 2.0 * m * cosf(phi + TWO_PI_3);
            roots[2] = 2.0 * m * cosf(phi - TWO_PI_3);
        }
        // Cardano loses precision when the cubic term is small; polish each
        // root with Newton steps in f64 (libskybrush does not do this)
        let cs = [d as f64, c as f64, b as f64, a as f64];
        for r in roots[..n].iter_mut() {
            let mut t = (*r + offset) as f64;
            for _ in 0..3 {
                let f = ((cs[3] * t + cs[2]) * t + cs[1]) * t + cs[0];
                let df = (3.0 * cs[3] * t + 2.0 * cs[2]) * t + cs[1];
                if df == 0.0 || !(f / df).is_finite() {
                    break;
                }
                t -= f / df;
            }
            *r = t as f32;
        }
        n
    }

    /// Smallest t in [0, 1] with p(t) == value. Degree > 3 never touches,
    /// matching libskybrush.
    pub fn touches(&self, value: f32) -> Option<f32> {
        match self.significant_coeffs() {
            0 | 1 => (value == self.coeffs[0]).then_some(0.0),
            2 => self.touches_2d(value),
            3 => self.touches_3d(value),
            4 => self.touches_4d(value),
            _ => None,
        }
    }

    fn touches_2d(&self, value: f32) -> Option<f32> {
        let a = self.coeffs[1];
        let b = self.coeffs[0];
        if is_zero(a) {
            (value == self.coeffs[0]).then_some(0.0)
        } else if (a > 0.0 && value >= b && value <= a + b) || (a < 0.0 && value >= a + b && value <= b) {
            Some((value - b) / a)
        } else {
            None
        }
    }

    fn leftmost_root(&self, value: f32) -> Option<f32> {
        let mut roots = [0.0f32; 3];
        let n = self.solve(value, &mut roots).ok()?;
        let mut best = 2.0f32;
        for &r in &roots[..n] {
            if r >= 0.0 && r <= 1.0 && r < best {
                best = r;
            }
        }
        (best < 2.0).then_some(best)
    }

    fn touches_3d(&self, value: f32) -> Option<f32> {
        let a = self.coeffs[2];
        let b = self.coeffs[1];
        let c = self.coeffs[0];
        if c == value {
            return Some(0.0);
        }
        if a + b + c == value {
            return Some(1.0);
        }
        if c > value && b >= 0.0 && a >= -b / 2.0 {
            return None;
        }
        if c < value && b <= 0.0 && a <= -b / 2.0 {
            return None;
        }
        self.leftmost_root(value)
    }

    fn touches_4d(&self, value: f32) -> Option<f32> {
        let a = self.coeffs[3];
        let b = self.coeffs[2];
        let c = self.coeffs[1];
        let d = self.coeffs[0];
        if d == value {
            return Some(0.0);
        }
        if a + b + c + d == value {
            return Some(1.0);
        }
        let dend = 3.0 * a + 2.0 * b + c;
        if d > value && c >= 0.0 && dend >= 0.0 {
            if a <= 0.0 || b > 0.0 || b < -3.0 * a {
                return None;
            }
            if c - (b * b) / (3.0 * a) >= 0.0 {
                return None;
            }
        }
        if d < value && c <= 0.0 && dend <= 0.0 {
            if a >= 0.0 || b < 0.0 || b > -3.0 * a {
                return None;
            }
            if c - (b * b) / (3.0 * a) <= 0.0 {
                return None;
            }
        }
        self.leftmost_root(value)
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Poly3 {
    pub x: Poly,
    pub y: Poly,
    pub z: Poly,
}

impl Poly3 {
    pub fn constant(v: Vec3) -> Self {
        Poly3 { x: Poly::constant(v.x), y: Poly::constant(v.y), z: Poly::constant(v.z) }
    }

    pub fn eval(&self, t: f32) -> Vec3 {
        Vec3::new(self.x.eval(t), self.y.eval(t), self.z.eval(t))
    }

    pub fn deriv(&mut self) {
        self.x.deriv();
        self.y.deriv();
        self.z.deriv();
    }

    pub fn scale(&mut self, k: f32) {
        self.x.scale(k);
        self.y.scale(k);
        self.z.scale(k);
    }
}

/// Control points of the part of a Bezier curve before `ratio` (De Casteljau).
pub fn bezier_cut_at(dst: &mut [f32], src: &[f32], ratio: f32) -> Result<()> {
    let n = src.len();
    if !(0.0..=1.0).contains(&ratio) || n > 8 {
        return Err(Error::Inval);
    }
    if ratio == 0.0 {
        dst[..n].fill(src[0]);
    } else if ratio == 1.0 {
        dst[..n].copy_from_slice(src);
    } else {
        let mut temp = [0.0f32; 8];
        temp[..n].copy_from_slice(src);
        dst[0] = temp[0];
        for i in 1..n {
            for j in 0..n - i {
                temp[j] = (1.0 - ratio) * temp[j] + ratio * temp[j + 1];
            }
            dst[i] = temp[0];
        }
    }
    Ok(())
}
