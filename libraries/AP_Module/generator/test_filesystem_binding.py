# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate bounded filesystem contracts, SDK wire and descriptor cleanup."""

import copy
import ctypes
import os
import struct
import subprocess

from pathlib import Path

import bindings
import filesystem_binding as filesystem
import pytest


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert filesystem.select(ir, 700) == filesystem.specification(700)
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['singletons'] if item['name'] == filesystem.OWNER)
    next(item for item in owner['methods'] if item['name'] == 'stat')['arguments'][1]['nullable'] = False
    with pytest.raises(ValueError, match='stat arguments'):
        filesystem.select(changed, 700)


def test_contract_is_bounded_and_handle_free():
    spec = filesystem.specification(700)
    assert [method['id'] for method in spec['methods']] == list(range(700, 707))
    assert [method['name'] for method in spec['methods']] == [
        'filesystem.stat', 'filesystem.read_at', 'filesystem.dir_entry',
        'filesystem.truncate', 'filesystem.append', 'filesystem.unlink', 'filesystem.format',
    ]
    assert 'no-retained-descriptors' in spec['methods'][0]['policy']
    assert spec['read_max'] == 1024
    assert spec['write_max'] == 1024


@pytest.fixture
def host_service(tmp_path):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': '',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t) { return true; } void give() {} }; }
using HAL_Semaphore = AP_HAL::Semaphore;
''',
        'AP_Filesystem/AP_Filesystem.h': r'''#pragma once
#include <stdint.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <dirent.h>
class AP_Filesystem {
public:
    struct stat_t { uint32_t size; int32_t mode; uint32_t mtime,atime,ctime;
                    bool is_directory() const { return (mode&S_IFMT)==S_IFDIR; } };
    bool stat(const char *, stat_t &); int open(const char *, int); int close(int);
    int32_t lseek(int, int32_t, int); int32_t read(int, void *, uint32_t);
    int32_t write(int, const void *, uint32_t); int unlink(const char *);
    bool format();
    struct Handle { unsigned index; };
    Handle *opendir(const char *); struct dirent *readdir(Handle *); int closedir(Handle *);
};
namespace AP { AP_Filesystem &FS(); }
''',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)
    service = Path(__file__).parents[1] / 'BindingFilesystem.cpp'
    harness = tmp_path / 'host.cpp'
    harness.write_text(f'''#include <string.h>
#include "{service}"
static unsigned opens, closes, dirs, closedirs, unlinks;
static unsigned formats;
static uint32_t position;
static uint8_t written_data[2048]; static uint16_t written_length; static bool append_open;
AP_Filesystem &AP::FS() {{ static AP_Filesystem fs; return fs; }}
bool AP_Filesystem::stat(const char *path, stat_t &s) {{
    if (!strcmp(path,"missing")) return false;
    s={{9, !strcmp(path,"dir") ? S_IFDIR : S_IFREG, 123,0,0}}; return true;
}}
int AP_Filesystem::open(const char *path,int flags) {{
    if (!strcmp(path,"missing")) return -1;
    if (flags==O_RDONLY) {{ opens++; append_open=false; return 7; }}
    if (flags==(O_WRONLY|O_CREAT|O_TRUNC)) {{
        opens++; append_open=false; written_length=0; return 7;
    }}
    if (flags==(O_WRONLY|O_CREAT|O_APPEND)) {{ opens++; append_open=true; return 7; }}
    __builtin_trap();
}}
int AP_Filesystem::close(int fd) {{ if(fd!=7)__builtin_trap(); closes++; return 0; }}
int32_t AP_Filesystem::lseek(int fd,int32_t p,int whence) {{
    if(fd!=7||whence!=SEEK_SET) {{ __builtin_trap(); }}
    position=p; return p;
}}
int32_t AP_Filesystem::read(int fd,void *out,uint32_t count) {{
    if(fd!=7) {{ __builtin_trap(); }}
    if(position>=9) return 0;
    unsigned n=count<9-position?count:9-position;
    memcpy(out,&"123456789"[position],n); position+=n; return n;
}}
int32_t AP_Filesystem::write(int fd,const void *data,uint32_t count) {{
    if(fd!=7||!append_open||written_length+count>sizeof(written_data)) {{ __builtin_trap(); }}
    uint32_t amount=count>2?2:count;
    memcpy(written_data+written_length,data,amount); written_length+=amount; return amount;
}}
int AP_Filesystem::unlink(const char *path) {{
    if(strcmp(path,"remove")) return -1;
    unlinks++; return 0;
}}
bool AP_Filesystem::format() {{ formats++; return true; }}
AP_Filesystem::Handle *AP_Filesystem::opendir(const char *path) {{
    if(strcmp(path,"dir")) return nullptr;
    dirs++; return new Handle{{0}};
}}
struct dirent *AP_Filesystem::readdir(Handle *h) {{
    static struct dirent e; const char *names[]={{"one","two"}};
    if(h->index>=2) return nullptr;
    memset(&e,0,sizeof(e)); strcpy(e.d_name,names[h->index++]); return &e;
}}
int AP_Filesystem::closedir(Handle *h) {{ closedirs++; delete h; return 0; }}
extern "C" int stat_call(const void *in,void *out) {{
 return AP_Module::binding_filesystem_stat((const uint8_t*)in,256,(uint8_t*)out,10); }}
extern "C" int read_call(const void *in,void *out) {{
 return AP_Module::binding_filesystem_read_at((const uint8_t*)in,262,(uint8_t*)out,1027); }}
extern "C" int dir_call(const void *in,void *out) {{
 return AP_Module::binding_filesystem_dir_entry((const uint8_t*)in,258,(uint8_t*)out,258); }}
extern "C" int truncate_call(const void *in,void *out) {{
 return AP_Module::binding_filesystem_truncate((const uint8_t*)in,256,(uint8_t*)out,1); }}
extern "C" int append_call(const void *in,void *out) {{
 return AP_Module::binding_filesystem_append((const uint8_t*)in,1282,(uint8_t*)out,1); }}
extern "C" int unlink_call(const void *in,void *out) {{
 return AP_Module::binding_filesystem_unlink((const uint8_t*)in,256,(uint8_t*)out,1); }}
extern "C" int format_call(void *out) {{
 return AP_Module::binding_filesystem_format(nullptr,0,(uint8_t*)out,1); }}
extern "C" unsigned live_files() {{ return opens-closes; }}
extern "C" unsigned live_dirs() {{ return dirs-closedirs; }}
extern "C" unsigned write_length() {{ return written_length; }}
extern "C" const uint8_t *write_data() {{ return written_data; }}
extern "C" unsigned unlink_count() {{ return unlinks; }}
extern "C" unsigned format_count() {{ return formats; }}
''')
    library = tmp_path / 'host.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-fPIC', '-shared', '-DAP_MODULE_ENABLED=1',
                    '-DAP_FILESYSTEM_FILE_READING_ENABLED=1', '-DAP_FILESYSTEM_FILE_WRITING_ENABLED=1',
                    '-DAP_FILESYSTEM_FORMAT_ENABLED=1',
                    '-I' + str(tmp_path),
                    '-I' + str(bindings.ROOT), str(harness), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    for name in ('stat_call', 'read_call', 'dir_call', 'truncate_call', 'append_call', 'unlink_call'):
        getattr(lib, name).argtypes = [ctypes.c_void_p, ctypes.c_void_p]
    lib.format_call.argtypes = [ctypes.c_void_p]
    lib.write_data.restype = ctypes.POINTER(ctypes.c_uint8)
    return lib


def path_wire(path):
    return path.encode().ljust(256, b'\0')


def test_host_stat_read_and_dir_close_every_resource(host_service):
    lib = host_service
    output = ctypes.create_string_buffer(1027)
    assert lib.stat_call(path_wire('file'), output) == 0
    assert output.raw[:10] == b'\1' + struct.pack('<II?', 9, 123, False)
    packet = path_wire('@SYS/file') + struct.pack('<IH', 2, 4)
    assert lib.read_call(packet, output) == 0
    assert output.raw[:7] == b'\1\4\0' + b'3456'
    packet = path_wire('dir') + struct.pack('<H', 1)
    assert lib.dir_call(packet, output) == 0
    assert output.raw[:6] == b'\1\3\0two'
    assert lib.live_files() == 0
    assert lib.live_dirs() == 0


def test_host_truncate_append_unlink_are_bounded_and_stateless(host_service):
    lib = host_service
    one = ctypes.create_string_buffer(1)
    assert lib.truncate_call(path_wire('copy'), one) == 0 and one.raw == b'\1'
    for data in (b'abc', b'defg'):
        packet = path_wire('copy') + struct.pack('<H', len(data)) + data.ljust(1024, b'\0')
        assert lib.append_call(packet, one) == 0 and one.raw == b'\1'
        assert lib.live_files() == 0
    assert lib.write_length() == 7
    assert bytes(lib.write_data()[:7]) == b'abcdefg'

    # Nonzero tail bytes are rejected before opening a descriptor.
    packet = path_wire('copy') + struct.pack('<H', 1) + b'x' + b'y' + bytes(1022)
    assert lib.append_call(packet, one) == -1
    assert lib.live_files() == 0
    assert lib.unlink_call(path_wire('remove'), one) == 0 and one.raw == b'\1'
    assert lib.unlink_count() == 1
    assert lib.unlink_call(path_wire('missing'), one) == 0 and one.raw == b'\0'
    assert lib.format_call(one) == 0 and one.raw == b'\1'
    assert lib.format_count() == 1


def test_emitted_sdk_compiles(tmp_path):
    generated = filesystem.rust(filesystem.specification(700), '')
    assert "impl crate::bindings::Fs<'_, '_>" in generated
    source = tmp_path / 'sdk.rs'
    imports = '\n'.join(f'pub(crate) static BINDING_{index}: Import = Import({index});'
                        for index in range(700, 707))
    source.write_text('''
use core::ffi::c_void;
#[derive(Clone,Copy,Debug,Eq,PartialEq)] pub enum Error { Invalid, Host }
pub type Result<T=()> = core::result::Result<T,Error>;
fn result(s:i32)->Result { if s==0 {Ok(())} else {Err(Error::Host)} }
struct HostApi { binding_call: Option<unsafe extern "C" fn(*mut c_void,u32,*const u8,u32,*mut u8,u32)->i32> }
pub struct Context<'a>{api:&'a HostApi,raw:*mut c_void}
pub mod bindings {
    use super::Context;
    pub struct Fs<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
pub(crate) struct Import(u32); impl Import {pub(crate) fn slot(&self)->u32{self.0}}
pub(crate) mod imports {use super::Import;
''' + imports + '\n}\nmod generated {\n' + generated + '\n}\nfn main() {}\n')
    subprocess.run(['rustc', '--edition=2021', str(source), '-o', str(tmp_path / 'sdk')], check=True)
