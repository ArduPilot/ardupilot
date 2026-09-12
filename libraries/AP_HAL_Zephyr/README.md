# ArduPilot on Zephyr RTOS

AP_HAL_Zephyr runs ArduPilot on top of the [Zephyr RTOS](https://www.zephyrproject.org/).
Instead of driving STM32 peripherals directly the way AP_HAL_ChibiOS does, it
sits on Zephyr's device drivers, so any SoC with decent Zephyr support becomes
a candidate ArduPilot target.

Bringing up a new board is then mostly devicetree and Kconfig work rather than
writing peripheral drivers from scratch. It also gets us onto architectures
ArduPilot hasn't traditionally run on - the boards below cover Cortex-M7
and Xtensa, plus a host build.

If you already know AP_HAL_ChibiOS, read [COMPARED_TO_CHIBIOS.md](COMPARED_TO_CHIBIOS.md).

## Boards

| waf board          | SoC                | Architecture | Zephyr board target             |
| ------------------ | ------------------ | ------------ | ------------------------------- |
| `mr_vmu_rt1176`    | NXP i.MX RT1176    | Cortex-M7    | `mr_vmu_rt1176/mimxrt1176/cm7`  |
| `CubeOrangeZephyr` | ST STM32H743       | Cortex-M7    | `cube_orange_zephyr`            |
| `ESP32S3Zephyr`    | Espressif ESP32-S3 | Xtensa LX7   | `esp32s3_zephyr/esp32s3/procpu` |
| `native_sim`       | host               | x86-64 Linux | `native_sim/native/64`          |

CubeOrangeZephyr is the same hardware as the ChibiOS CubeOrange target, on
purpose. Flashing both to one board is the only honest way to compare the two
backends, and it's where most of the numbers in COMPARED_TO_CHIBIOS.md came
from.

native_sim builds the lot as a Linux executable. It runs the real HAL code
path with no hardware attached, which makes it the cheapest way to check we
haven't broken boot, storage or the main loop. It is not SITL - SITL replaces
the HAL, native_sim exercises it.

## Building

```sh
./Tools/scripts/zephyr_get_prerequisites.sh     # first time only
./waf configure --board=mr_vmu_rt1176      # or CubeOrangeZephyr, ESP32S3Zephyr, ...
./waf copter -j12
```

Any board from the table works in place of `mr_vmu_rt1176`. Where the same
hardware also has a ChibiOS target, the Zephyr one is that name with `Zephyr`
appended: `CubeOrange` becomes `CubeOrangeZephyr`.

native_sim gives you something you can run directly:

```sh
./waf configure --board=native_sim && ./waf copter
./build/native_sim/zephyr_build/zephyr/zephyr.exe
```

Two build behaviours differ from a normal ArduPilot tree:

**Edit `hwdef.dat` and you must re-run `waf configure`.** It regenerates
`hwdef.h`, but nothing declares a dependency from AP sources to that generated
header, so a plain rebuild keeps the old contents and says nothing. The
symptom is a sensor that stubbornly refuses to appear.

**`waf clean` is refused on Zephyr boards.** It deletes files inside
`modules/zephyr` and the generated `hwdef.h`. Delete `build/<board>/` by hand.
Other board classes clean normally.

## How the build fits together

Waf stays in charge. It doesn't replace Zephyr's build, it drives it:

1. `waf configure` picks the board, sets `ZEPHYR_BOARD`, and turns
   `hwdef/<board>/hwdef.dat` into a generated `hwdef.h`.
2. `waf build` runs CMake and Ninja over `libraries/AP_HAL_Zephyr/zephyr/`,
   which is an ordinary Zephyr application, and gets `libzephyr.a` out.
3. ArduPilot's own libraries are compiled by waf as usual and linked against
   `libzephyr.a`.

We want the smallest Zephyr that will do the job, not a full Zephyr
application, so most of its subsystems stay switched off.

### Kconfig fragments

Fragments are merged most-general-first, each layer overriding the last:

```text
zephyr/prj.conf                 base, all boards
zephyr/prj.<manufacturer>.conf  prj.nxp.conf
zephyr/prj.<soc>.conf           prj.nxprt1176.conf
zephyr/prj.<board>.conf         prj.mr_vmu_rt1176.conf
```

After those comes a generated overlay that only fills in symbols nothing
earlier set, and then whatever a configure flag asked for.

The merged result lands in
`build/<board>/zephyr_build/ardupilot_prj_autogen.conf`. **If a symbol you set
isn't in that file, Kconfig never saw it.** Check there first when a config
change appears to do nothing.

Two flags add a final fragment:

- `./waf configure --ship` merges `ship.conf` last and forces every
  `CONFIG_AP_*` diagnostic off - the SPI and I2C probe scans, the scheduler
  trace, and the delay, chain and ISR profilers. Crash dump capture and the
  storage backend stay on; those are features, not diagnostics. Use it for
  anything you intend to fly.
- `./waf configure --enable-stats` merges `thread_stats.conf` for per-thread
  CPU and stack accounting, at the cost of a timestamp read on every context
  switch. It's the opposite of `--ship`. Nothing stops you passing both and
  the last fragment just wins, so don't.

## No west

Zephyr and its dependencies are git submodules and pinned checkouts. You do
not need `west` installed to work on this.

`modules/zephyr` is a submodule. Zephyr's own dependency repos can't be, since
git won't track paths inside another submodule's gitlink, so the source of
truth for those is `Tools/scripts/zephyr_manifest_v4_4_0_map.tsv` - one row per
dependency with name, URL, commit and path.

`zephyr_get_prerequisites.sh` reads that map and creates every checkout, plus
installs host packages and fetches the Espressif RF blobs the ESP32 targets
link against, checking each against the SHA-256 in hal_espressif's own
`module.yml`.

## What lives where

| Path              | Contents                                                 |
| ----------------- | -------------------------------------------------------- |
| `*.cpp`, `*.h`    | the HAL itself, 43 files                                 |
| `hwdef/<board>/`  | sensors, buses and serial config per board               |
| `zephyr/`         | the Zephyr application: CMake, Kconfig, `prj*.conf`      |
| `zephyr/src/`     | Zephyr-side C glue and SoC fixups                        |
| `zephyr/boards/`  | devicetree and pinctrl for boards not upstream in Zephyr |

The HAL class is `HAL_Zephyr`, namespace `Zephyr::`. File layout and naming
track AP_HAL_ChibiOS deliberately closely so you can diff the two when
something behaves differently.

Per-board wiring and connector detail is in `hwdef/<board>/README.md`.
Debugging, tooling and crash dumps are in [DEBUGGING.md](DEBUGGING.md).
