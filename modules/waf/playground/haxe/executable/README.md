# Using `HLC` source generation with `clang`

## Environment
In this particular case, you need to have a distribution of `hashlink` in your system. After installing, you need to perform additional steps to pass required files for binary generation (in this case - to `clang`):

- either add hashlink's `lib` folder to `ctx.env.LIBPATH_HL`
- or replace `lib` folder with a symlink to hashlink's `lib` folder
- either add hashlink's `include` folder to `ctx.env.INCLUDES_HL`
- or replace `include` folder with a symlink to hashlink's `include` folder

## Targets
In this particular case, generated `.c` files are placed in separate `bin` subdirectory. This enhances your build transparency and allows you to add desired checks or perform additional operations with generated `.c` sources if needed, while keeping things in parallel. Keep this in mind if you're planning to extend your build layout with additional Haxe targets

## Running assembled binaries
Assuming that you have a `hashlink` distribution and all relevant system paths are adjusted, you could easily run your binary and see resulting output of `Main.hx:3: hello`. Keep in mind that if you're using an official `hashlink` distribution, it doesn't come with static libs for linking - this means that your produced binary requires paths to `libhl.dll` (or `.so`/`.dylib` - depends on your system). Of course, there may be a use case when you're building `hashlink` from sources or using it as a portable distribution - in these cases, you could run your binary while pointing paths to your dynamic libraries with adding correct paths (`$PWD/lib/` for example) to:

- `PATH` on windows
- `LD_LIBRARY_PATH` on linux
- `DYLD_LIBRARY_PATH` on macOS