## littlefs

A little fail-safe filesystem designed for microcontrollers.

```
   | | |     .---._____
  .-----.   |          |
--|o    |---| littlefs |
--|     |---|          |
  '-----'   '----------'
   | | |
```

**Power-loss resilience** - littlefs is designed to handle random power
failures. All file operations have strong copy-on-write guarantees and if
power is lost the filesystem will fall back to the last known good state.

**Dynamic wear leveling** - littlefs is designed with flash in mind, and
provides wear leveling over dynamic blocks. Additionally, littlefs can
detect bad blocks and work around them.

**Bounded RAM/ROM** - littlefs is designed to work with a small amount of
memory. RAM usage is strictly bounded, which means RAM consumption does not
change as the filesystem grows. The filesystem contains no unbounded
recursion and dynamic memory is limited to configurable buffers that can be
provided statically.

## Example

Here's a simple example that updates a file named `boot_count` every time
main runs. The program can be interrupted at any time without losing track
of how many times it has been booted and without corrupting the filesystem:

``` c
#include "lfs.h"

// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = user_provided_block_device_read,
    .prog  = user_provided_block_device_prog,
    .erase = user_provided_block_device_erase,
    .sync  = user_provided_block_device_sync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = 4096,
    .block_count = 128,
    .cache_size = 16,
    .lookahead_size = 16,
    .block_cycles = 500,
};

// entry point
int main(void) {
    // mount the filesystem
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err) {
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }

    // read current count
    uint32_t boot_count = 0;
    lfs_file_open(&lfs, &file, "boot_count", LFS_O_RDWR | LFS_O_CREAT);
    lfs_file_read(&lfs, &file, &boot_count, sizeof(boot_count));

    // update boot count
    boot_count += 1;
    lfs_file_rewind(&lfs, &file);
    lfs_file_write(&lfs, &file, &boot_count, sizeof(boot_count));

    // remember the storage is not updated until the file is closed successfully
    lfs_file_close(&lfs, &file);

    // release any resources we were using
    lfs_unmount(&lfs);

    // print the boot count
    printf("boot_count: %d\n", boot_count);
}
```

## Usage

Detailed documentation (or at least as much detail as is currently available)
can be found in the comments in [lfs.h](lfs.h).

littlefs takes in a configuration structure that defines how the filesystem
operates. The configuration struct provides the filesystem with the block
device operations and dimensions, tweakable parameters that tradeoff memory
usage for performance, and optional static buffers if the user wants to avoid
dynamic memory.

The state of the littlefs is stored in the `lfs_t` type which is left up
to the user to allocate, allowing multiple filesystems to be in use
simultaneously. With the `lfs_t` and configuration struct, a user can
format a block device or mount the filesystem.

Once mounted, the littlefs provides a full set of POSIX-like file and
directory functions, with the deviation that the allocation of filesystem
structures must be provided by the user.

All POSIX operations, such as remove and rename, are atomic, even in event
of power-loss. Additionally, file updates are not actually committed to
the filesystem until sync or close is called on the file.

## Other notes

Littlefs is written in C, and specifically should compile with any compiler
that conforms to the `C99` standard.

All littlefs calls have the potential to return a negative error code. The
errors can be either one of those found in the `enum lfs_error` in
[lfs.h](lfs.h), or an error returned by the user's block device operations.

In the configuration struct, the `prog` and `erase` function provided by the
user may return a `LFS_ERR_CORRUPT` error if the implementation already can
detect corrupt blocks. However, the wear leveling does not depend on the return
code of these functions, instead all data is read back and checked for
integrity.

If your storage caches writes, make sure that the provided `sync` function
flushes all the data to memory and ensures that the next read fetches the data
from memory, otherwise data integrity can not be guaranteed. If the `write`
function does not perform caching, and therefore each `read` or `write` call
hits the memory, the `sync` function can simply return 0.

## Design

At a high level, littlefs is a block based filesystem that uses small logs to
store metadata and larger copy-on-write (COW) structures to store file data.

In littlefs, these ingredients form a sort of two-layered cake, with the small
logs (called metadata pairs) providing fast updates to metadata anywhere on
storage, while the COW structures store file data compactly and without any
wear amplification cost.

Both of these data structures are built out of blocks, which are fed by a
common block allocator. By limiting the number of erases allowed on a block
per allocation, the allocator provides dynamic wear leveling over the entire
filesystem.

```
                    root
                   .--------.--------.
                   | A'| B'|         |
                   |   |   |->       |
                   |   |   |         |
                   '--------'--------'
                .----'   '--------------.
       A       v                 B       v
      .--------.--------.       .--------.--------.
      | C'| D'|         |       | E'|new|         |
      |   |   |->       |       |   | E'|->       |
      |   |   |         |       |   |   |         |
      '--------'--------'       '--------'--------'
      .-'   '--.                  |   '------------------.
     v          v              .-'                        v
.--------.  .--------.        v                       .--------.
|   C    |  |   D    |   .--------.       write       | new E  |
|        |  |        |   |   E    |        ==>        |        |
|        |  |        |   |        |                   |        |
'--------'  '--------'   |        |                   '--------'
                         '--------'                   .-'    |
                         .-'    '-.    .-------------|------'
                        v          v  v              v
                   .--------.  .--------.       .--------.
                   |   F    |  |   G    |       | new F  |
                   |        |  |        |       |        |
                   |        |  |        |       |        |
                   '--------'  '--------'       '--------'
```

More details on how littlefs works can be found in [DESIGN.md](DESIGN.md) and
[SPEC.md](SPEC.md).

- [DESIGN.md](DESIGN.md) - A fully detailed dive into how littlefs works.
  I would suggest reading it as the tradeoffs at work are quite interesting.

- [SPEC.md](SPEC.md) - The on-disk specification of littlefs with all the
  nitty-gritty details. May be useful for tooling development.

## Testing

The littlefs comes with a test suite designed to run on a PC using the
[emulated block device](bd/lfs_testbd.h) found in the `bd` directory.
The tests assume a Linux environment and can be started with make:

``` bash
make test
```

## License

The littlefs is provided under the [BSD-3-Clause] license. See
[LICENSE.md](LICENSE.md) for more information. Contributions to this project
are accepted under the same license.

Individual files contain the following tag instead of the full license text.

    SPDX-License-Identifier:    BSD-3-Clause

This enables machine processing of license information based on the SPDX
License Identifiers that are here available: http://spdx.org/licenses/

## Related projects

- [littlefs-fuse] - A [FUSE] wrapper for littlefs. The project allows you to
  mount littlefs directly on a Linux machine. Can be useful for debugging
  littlefs if you have an SD card handy.

- [littlefs-js] - A javascript wrapper for littlefs. I'm not sure why you would
  want this, but it is handy for demos.  You can see it in action
  [here][littlefs-js-demo].
  
- [littlefs-python] - A Python wrapper for littlefs. The project allows you
  to create images of the filesystem on your PC. Check if littlefs will fit
  your needs, create images for a later download to the target memory or
  inspect the content of a binary image of the target memory.

- [mklfs] - A command line tool built by the [Lua RTOS] guys for making
  littlefs images from a host PC. Supports Windows, Mac OS, and Linux.

- [Mbed OS] - The easiest way to get started with littlefs is to jump into Mbed
  which already has block device drivers for most forms of embedded storage.
  littlefs is available in Mbed OS as the [LittleFileSystem] class.

- [SPIFFS] - Another excellent embedded filesystem for NOR flash. As a more
  traditional logging filesystem with full static wear-leveling, SPIFFS will
  likely outperform littlefs on small memories such as the internal flash on
  microcontrollers.

- [Dhara] - An interesting NAND flash translation layer designed for small
  MCUs. It offers static wear-leveling and power-resilience with only a fixed
  _O(|address|)_ pointer structure stored on each block and in RAM.


[BSD-3-Clause]: https://spdx.org/licenses/BSD-3-Clause.html
[littlefs-fuse]: https://github.com/geky/littlefs-fuse
[FUSE]: https://github.com/libfuse/libfuse
[littlefs-js]: https://github.com/geky/littlefs-js
[littlefs-js-demo]:http://littlefs.geky.net/demo.html
[mklfs]: https://github.com/whitecatboard/Lua-RTOS-ESP32/tree/master/components/mklfs/src
[Lua RTOS]: https://github.com/whitecatboard/Lua-RTOS-ESP32
[Mbed OS]: https://github.com/armmbed/mbed-os
[LittleFileSystem]: https://os.mbed.com/docs/mbed-os/v5.12/apis/littlefilesystem.html
[SPIFFS]: https://github.com/pellepl/spiffs
[Dhara]: https://github.com/dlbeer/dhara
[littlefs-python]: https://pypi.org/project/littlefs-python/
