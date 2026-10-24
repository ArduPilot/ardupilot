/* GPL-3.0-or-later. Offline link layout for the experimental module format. */
ENTRY(start)
SECTIONS
{
    . = DEFINED(MODULE_BASE) ? MODULE_BASE : 0x10000;
    /* Panic-abort modules never unwind. Discard compiler-builtins' unwind
       metadata before .data.* can retain its unused personality reference. */
    /DISCARD/ : {
        *(.eh_frame*) *(.gcc_except_table*)
        *(.data.DW.ref.rust_eh_personality)
        *(.comment) *(.note*)
        /* Pretty-printers are registered by the host GDB helper, not loaded
           from target memory. Keep their bootstrap out of the packed image. */
        *(.debug_gdb_scripts)
    }
    .text : { *(.text .text.*) }
    .rodata : { *(.rodata .rodata.*) }
    .ARM.exidx : { *(.ARM.exidx .ARM.exidx.*) }
    /* ARM split images need only word separation. Keep the default page
       boundary for POSIX permission changes and legacy relocation probes. */
    . = ALIGN(MAX(DEFINED(MODULE_DATA_ALIGNMENT) ? MODULE_DATA_ALIGNMENT : 4096,
                  MAX(ALIGNOF(.data), MAX(ALIGNOF(.got), ALIGNOF(.bss)))));
    .data : {
        *(.data .data.*)
        *(.data.rel.ro .data.rel.ro.*)
        /* Do not KEEP these: unused wrappers and imports must be collected. */
        *(.ap_module_import_*)
    }
    .got : { *(.got) }
    .bss (NOLOAD) : { *(.bss .bss.*) *(COMMON) }
}
