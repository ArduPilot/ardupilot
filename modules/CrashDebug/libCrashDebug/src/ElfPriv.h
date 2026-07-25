/*  Copyright (C) 2014  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* ELF file format types, structures, etc, required for FLASH loading. Taken from ELF format documentation. */
#ifndef _ELF_PRIV_H_
#define _ELF_PRIV_H_


#include <stdint.h>


/* Elf32_Ehdr::e_type values */
#define ET_NONE     0       /* No file type */
#define ET_REL      1       /* Relocatable file */
#define ET_EXEC     2       /* Executbale file */
#define ET_DYN      3       /* Shared object file */
#define ET_CORE     4       /* Core file */
#define ET_LOPPROC  0xff00  /* Processor-specific */
#define ET_HIPROC   0xffff  /* Processor-specific */

/* Elf32_Ehdr::e_machine values */
#define EM_NONE 0   /* No machine */
#define EM_M32  1   /* AT&T WE 32100 */
#define EM_SPARC    2   /* SPARC */
#define EM_386      3   /* Intel 80386 */
#define EM_68K      4   /* Motorola 68000 */
#define EM_88K      5   /* Motorola 88000 */
#define EM_860      7   /* Intel 8060 */
#define EM_MIPS     8   /* MIPS RS3000 */
#define EM_ARM      40  /* ARM */

/* Elf32_Ehdr::e_version values */
#define EV_NONE     0   /* Invalid version */
#define EV_CURRENT  1   /* Current version */

/* Elf32_Ehdr::e_ident byte indices */
#define EI_MAG0     0   /* File identification */
#define EI_MAG1     1
#define EI_MAG2     2
#define EI_MAG3     3
#define EI_CLASS    4   /* File class */
#define EI_DATA     5   /* Data encoding */
#define EI_VERSION  6   /* File version */
#define EI_PAD      7   /* Start of padding bytes */
#define EI_NIDENT   16  /* Size of e_ident[] */

/* Values expected to be Elf32_Ehdr::e_ident[EI_MAG0..EI_MAG3] */
#define ELFMAG0 0x7f
#define ELFMAG1 'E'
#define ELFMAG2 'L'
#define ELFMAG3 'F'

/* Values for Elf32_Ehdr::e_ident[EI_CLASS] */
#define ELFCLASSNONE    0   /* Invalid class */
#define ELFCLASS32      1   /* 32-bit objects */
#define ELFCLASS64      2   /* 64-bit objects */

/* Values for Elf32_Ehdr::e_ident[EI_DATA] */
#define ELFDATANONE 0   /* Invalid data encoding */
#define ELFDATA2LSB 1   /* 2's complement least significant byte endian ordering */
#define ELFDATA2MSB 2   /* 2's complement most significant byte endian ordering */

/* Values for Elf32_Phdr::p_type */
#define PT_NULL     0
#define PT_LOAD     1
#define PT_DYNAMIC  2
#define PT_INTERP   3
#define PT_NOTE     4
#define PT_SHLIB    5
#define PT_PHDR     6
#define PT_LOPROC   0x70000000
#define PT_HIPROC   0x7fffffff

/* Bits for Elf32_Phdr::p_flags bitmask */
#define PF_R    4
#define PF_W    2
#define PF_X    1

typedef uint32_t Elf32_Addr;
typedef uint16_t Elf32_Half;
typedef uint32_t Elf32_Off;
typedef int32_t  Elf32_Sword;
typedef uint32_t Elf32_Word;

/* ELF File Header */
typedef struct
{
    unsigned char   e_ident[EI_NIDENT];
    Elf32_Half      e_type;
    Elf32_Half      e_machine;
    Elf32_Word      e_version;
    Elf32_Addr      e_entry;
    Elf32_Off       e_phoff;
    Elf32_Off       e_shoff;
    Elf32_Word      e_flags;
    Elf32_Half      e_ehsize;
    Elf32_Half      e_phentsize;
    Elf32_Half      e_phnum;
    Elf32_Half      e_shentsize;
    Elf32_Half      e_shnum;
    Elf32_Half      e_shstrndx;
} Elf32_Ehdr;

/* ELF Program Header */
typedef struct
{
    Elf32_Word p_type;
    Elf32_Off  p_offset;
    Elf32_Addr p_vaddr;
    Elf32_Addr p_paddr;
    Elf32_Word p_filesz;
    Elf32_Word p_memsz;
    Elf32_Word p_flags;
    Elf32_Word p_align;
} Elf32_Phdr;


#endif /* _ELF_PRIV_H_ */
