# ArduPilot on Zephyr RTOS

AP_HAL_Zephyr runs ArduPilot on top of the [Zephyr RTOS](https://www.zephyrproject.org/).
Instead of driving STM32 peripherals directly the way AP_HAL_ChibiOS does, it
sits on Zephyr's device drivers, so any SoC with decent Zephyr support becomes
a candidate ArduPilot target.

Bringing up a new board is then mostly devicetree and Kconfig work rather than
writing peripheral drivers from scratch. ArduPilot already reaches Cortex-M7
through AP_HAL_ChibiOS and Xtensa through AP_HAL_ESP32, so neither is new
ground on its own; what is different here is that one HAL covers both families
and a host build from one set of sources, because the per-peripheral driver
work belongs to Zephyr.

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
./Tools/zephyr/zephyr_get_prerequisites.sh     # first time only
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

Four build behaviours differ from a normal ArduPilot tree:

**Edit `hwdef.dat` and you must re-run `waf configure`.** It regenerates
`hwdef.h`, but nothing declares a dependency from AP sources to that generated
header, so a plain rebuild keeps the old contents and says nothing. The
symptom is a sensor that stubbornly refuses to appear.

**`waf clean` is refused on Zephyr boards.** It deletes files inside
`modules/zephyr` and the generated `hwdef.h`. Delete `build/<board>/` by hand.
Other board classes clean normally.

**`build/<board>/bin/` stays empty.** Waf compiles ArduPilot into static
archives and Zephyr's CMake does the final link, so nothing lands where a
ChibiOS build puts it:

| Artefact                      | Path                                                                       |
| ----------------------------- | -------------------------------------------------------------------------- |
| linked firmware               | `build/<board>/zephyr_build/zephyr/zephyr.elf` and `zephyr.bin`             |
| uploadable image              | `build/<board>/zephyr_upload.apj`                                          |
| MCUBoot slot-1 app image      | `build/<board>/ap_firmware_<board>.img` (some boards, see below)           |
| bootloader (`--bootloader`)   | `build/<board>/ap_bootloader_<board>.bin`, `.apj`, `.img`                  |
| native_sim executable         | `build/native_sim/zephyr_build/zephyr/zephyr.exe`                          |
| the AP archives waf built     | `build/<board>/lib/lib<Vehicle>_libs.a`, `build/<board>/lib/bin/lib<vehicle>.a` |

The `.apj` name is fixed, so it is one per build directory and not one per
vehicle (`_resolve_upload_firmware` in `Tools/ardupilotwaf/zephyr.py`). Build
copter and then plane in the same directory and the second silently overwrites
the first. ChibiOS writes `bin/arducopter.apj`, `bin/arduplane.apj`.

The MCUBoot app image is emitted only for a board whose app is linked at an
offset above a bootloader, which today is `mr_vmu_rt1176` alone; the other
boards get no `ap_firmware_<board>.img`. Both `.img` rows also need `imgtool`
on `PATH` - without it the build says so and writes the `.bin` and `.apj`
only.

**Build a bootloader before the vehicle, never after.** Two boards have a
`hwdef-bl.dat`, CubeOrangeZephyr and mr_vmu_rt1176, and their built
bootloaders are committed under `Tools/bootloaders/`:

```sh
./waf configure --board=mr_vmu_rt1176 --bootloader
./waf bootloader
./waf configure --board=mr_vmu_rt1176
./waf copter -j12
```

There is one `zephyr.elf` per build directory, so the vehicle build on the
last line overwrites the bootloader's `zephyr.elf` and `zephyr.bin`. The
`ap_bootloader_<board>.bin`, `.apj` and `.img` copies are named apart and
survive; the `.elf` is not, so copy it out before building the vehicle if you
want it for debugging. That is what the CI workflow does.

A build directory that has already held a vehicle build cannot be turned back
into a bootloader build by re-running `configure`. It fails with a
`uavcan.*.h: No such file` that is not true - the generated DroneCAN headers
are there. Delete `build/<board>/` and start from the first line again.

## How the build fits together

Waf stays in charge. It doesn't replace Zephyr's build, it drives it:

1. `waf configure` picks the board, sets `ZEPHYR_BOARD`, and turns
   `hwdef/<board>/hwdef.dat` into a generated `hwdef.h`.
2. A first CMake and Ninja pass over `libraries/AP_HAL_Zephyr/zephyr/`, which
   is an ordinary Zephyr application, produces `devicetree_generated.h`, the
   include paths every ArduPilot object compiles against, and `libzephyr.a`.
   `ARDUPILOT_LIB` is deliberately not passed here: the AP archives don't
   exist yet.
3. Waf compiles ArduPilot's own libraries as usual, into
   `build/<board>/lib/lib<Vehicle>_libs.a` and
   `build/<board>/lib/bin/lib<vehicle>.a`.
4. A second CMake configure passes `ARDUPILOT_LIB`, `ARDUPILOT_BIN` and
   `ARDUPILOT_CMD`, and `cmake --build` links `zephyr.elf`. The final link is
   Zephyr's, not waf's (`_run_ap_final_link` in
   `Tools/ardupilotwaf/zephyr.py`), which is why `build/<board>/bin/` is empty.
5. Waf converts the linked image and writes the `.apj`.

We want the smallest Zephyr that will do the job, not a full Zephyr
application, so most of its subsystems stay switched off.

### Kconfig fragments

Every fragment is concatenated into one file, and that file is what Zephyr
gets as `CONF_FILE`. A later assignment overrides an earlier one, so the order
is most-general first:

| Order | Fragment                                         | Example                             |
| ----- | ------------------------------------------------ | ----------------------------------- |
| 1     | `zephyr/prj.conf`                                | base, all boards                    |
| 2     | `zephyr/prj.<manufacturer>.conf`                 | `prj.nxp.conf`                      |
| 3     | `zephyr/prj.<soc>.conf`                          | `prj.nxprt1176.conf`                |
| 4     | `zephyr/prj.<board>.conf`                        | `prj.mr_vmu_rt1176.conf`            |
| 5     | `zephyr/boards/<board>.conf`                     | `boards/cube_orange_zephyr.conf`    |
| 6     | `zephyr/prj-bl.conf`, `zephyr/prj.<board>-bl.conf` | `--bootloader` builds only         |
| 7     | the fragment a configure flag asked for          | `thread_stats.conf`                 |
| 8     | `hwdef_autogen.conf`                             | generated from `hwdef.dat`          |

Layers 4 and 5 are both the board layer. Each name is tried against the waf
board name and against the Zephyr board target, and `zephyr/boards/` also
accepts a variant suffix (`_discover_zephyr_conf_fragments` in
`Tools/ardupilotwaf/zephyr.py`). A board can use either or both, so if you go
looking for a board's configuration you have to check both directories.
CubeOrangeZephyr uses both: `prj.CubeOrangeZephyr.conf` matched on the waf
name and `boards/cube_orange_zephyr.conf` matched on the Zephyr target. The
two names are tried per token rather than per layer, so 4-before-5 is the
order the boards here happen to get, not a rule. native_sim has no
`prj.native_sim.conf` at all, and its only board layer is
`zephyr/boards/native_sim_native_64.conf`, matched as a variant of
`native_sim`.

The match is on the whole token, so a file in `zephyr/boards/` whose name is
neither of a board's two tokens nor a variant of one is not merged at all.
There is one of those in the tree right now:
`zephyr/boards/esp32s3_zephyr.conf` is matched by nothing, because
ESP32S3Zephyr's two tokens are `ESP32S3Zephyr` and
`esp32s3_zephyr/esp32s3/procpu`. Zephyr does not pick it up either - waf sets
`CONF_FILE` explicitly, and Zephyr only scans an application's `boards/`
directory itself when `CONF_FILE` is unset. The only way to notice is that the
file's symbols are missing from the merged file below.

Layer 8 is a Kconfig fragment, not a devicetree overlay - the same generator
emits the overlay separately, as `hwdef_autogen.overlay`. The fragment only
assigns symbols no earlier fragment set, and on the boards in the table today
it is empty apart from its header comment.

The merged result lands in
`build/<board>/zephyr_build/ardupilot_prj_autogen.conf`. **If a symbol you set
in a `prj*.conf` isn't in that file, Kconfig never saw it.** Check there first
when a config change appears to do nothing.

That rule is about the fragments only, and there are two ways a symbol gets
its value without passing through them:

* **A board defconfig is a separate path.** `zephyr/boards/<arch>/<board>/`
  holds `<board>_defconfig` and `Kconfig.defconfig`, which Zephyr reads
  directly. `CONFIG_CORTEX_M_SYSTICK=y` reaches the rt1176 build from
  `zephyr/boards/arm/mr_vmu_rt1176/mr_vmu_rt1176_mimxrt1176_cm7_defconfig` and
  appears nowhere in the merged file.
* **A `select` beats a fragment.** `native_sim`'s board fragment sets
  `CONFIG_PRINTK=n` and the build still gets `CONFIG_PRINTK=y`, because
  `BOOT_BANNER` selects it (`modules/zephyr/lib/os/Kconfig`); `prj.conf` says
  so in a comment and assigns `y` for honesty.

So the merged file tells you what Kconfig was asked for, and
`build/<board>/zephyr_build/zephyr/.config` tells you what it decided. When
the two disagree, the `.config` is the answer.

One configure flag adds a fragment:

* `./waf configure --enable-stats` merges `thread_stats.conf` for per-thread
  CPU and stack accounting, at the cost of a timestamp read on every context
  switch.

## No west

Zephyr and its dependencies are git submodules and pinned checkouts. You do
not need `west` installed to work on this.

`modules/zephyr` is a submodule. Zephyr's own dependency repos can't be, since
git won't track paths inside another submodule's gitlink, so the source of
truth for those is `Tools/zephyr/zephyr_manifest_v4_4_0_map.tsv` - one row per
dependency with name, URL, commit and path.

`zephyr_get_prerequisites.sh` reads that map and creates every checkout, plus
installs host packages and fetches the Espressif RF blobs the ESP32 targets
link against, checking each against the SHA-256 in hal_espressif's own
`module.yml`.

## What lives where

| Path              | Contents                                                       |
| ----------------- | -------------------------------------------------------------- |
| `*.cpp`, `*.h`    | the HAL itself                                                  |
| `hwdef/<board>/`  | sensors, buses and serial config per board                      |
| `zephyr/`         | the Zephyr application: CMake, Kconfig, `prj*.conf`             |
| `zephyr/src/`     | Zephyr-side C glue and SoC fixups                               |
| `zephyr/boards/`  | devicetree, pinctrl and defconfigs for boards not upstream in Zephyr |
| `Tools/renode/`   | emulator board platforms, the C# peripheral models these boards need, and the flight harness CI runs them under |

The HAL class is `HAL_Zephyr`, namespace `Zephyr::`. File layout and naming
track AP_HAL_ChibiOS deliberately closely so you can diff the two when
something behaves differently.

`Tools/renode/` matters more here than on a ChibiOS board: CubeOrangeZephyr
and mr_vmu_rt1176 both boot under Renode, and
`.github/workflows/test_renode_zephyr.yml` flies a copter mission on each of
them, alongside a ChibiOS reference flight. Both its push and its pull-request
triggers are path-filtered, to `ArduCopter/`, `libraries/`, `modules/`,
`Tools/ardupilotwaf/`, `Tools/renode/`, `Tools/scripts/`, `waf`, `wscript` and
the workflow and actions themselves; a change that touches none of those runs
nothing. Read the job summary, not the tick: every flight step in that
workflow is `continue-on-error: true`, so a green job means the build and the
run completed, not that the mission passed. See
[Tools/renode/README.md](../../Tools/renode/README.md).

Per-board wiring and connector detail is in `hwdef/<board>/README.md` where
there is one - today that is `mr_vmu_rt1176` and `native_sim` only.
Debugging, tooling and crash dumps are in [DEBUGGING.md](DEBUGGING.md).
