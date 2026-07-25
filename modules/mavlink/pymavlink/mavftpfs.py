#!/usr/bin/env python3

import errno
import os
import time
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from typing import Any, Dict, Optional

from fuse import FUSE, FuseOSError, LoggingMixIn, Operations
from loguru import logger
from pymavlink import mavutil

from mavftp import MAVFTP

# this allows us to support accessing the special files
base_file_paths = ["/", "@ROMFS", "@SYS"]


class FTP(LoggingMixIn, Operations):
    def __init__(self, mav: Any) -> None:
        self.mav = mav
        self.missing_files = []
        self.cache_duration = 50
        self.files: Dict[str, Dict[str, int]] = {
            path: {
                "st_mode": 0o40755,
                "st_nlink": 2,
                "st_size": 0,
                # we need a timestamp for some systems to be happy
                "st_ctime": int(time.time()),
                "st_mtime": int(time.time()),
                "st_atime": int(time.time()),
                "ftp_last_update": 0,
            }
            for path in base_file_paths
        }
        self.ftp = MAVFTP(mav, target_system=mav.target_system, target_component=mav.target_component)

    def fix_path(self, path: str) -> str:
        if path.startswith("/@"):
            return path[1:]
        return path

    def getattr(self, path: str, _fh: int = 0) -> Any:
        path_fixed = self.fix_path(path)
        # logger.info(f"Fuse: getattr {path_fixed}")
        if path_fixed in self.missing_files:
            raise FuseOSError(errno.ENOENT)
        if path_fixed in self.files:
            return self.files[path_fixed]

        parent_dir = ("/" + "/".join(path.split("/")[:-1])).replace("//", "/")
        self.readdir(parent_dir)
        if path not in self.files:
            self.missing_files.append(path)
            raise FuseOSError(errno.ENOENT)
        return self.files[path]

    def read(self, path: str, size: int, offset: int, _fh: int = 0) -> Optional[bytes]:
        logger.info(f"Fuse: read {path}, size={size}, offset={offset}")
        buf = self.ftp.read_sector(self.fix_path(path), offset, size)
        return buf

    def cleaned_up_path(self, dir_path, current_path):
        if current_path == "/":
            return current_path
        if current_path.startswith(dir_path):
            remaining_path = current_path[len(dir_path):]
            if remaining_path.startswith("/"):
                return remaining_path[1:]
            return remaining_path


    def get_cached_dir_list(self, path: str) -> Dict[str, Dict[str, int]]:
        if self.files.get(path) is None:
            raise FuseOSError(errno.ENOENT)
        if not path.endswith("/"):
            path = path + "/"

        files_in_path = [file for file in self.files.keys() if file.startswith(path) and file != path and path.count("/") == file.count("/")]

        filtered_dict = {k: v for k, v in self.files.items() if k in files_in_path}

        remmaped_dict = {self.cleaned_up_path(path, k): v for k, v in filtered_dict.items()}

        return remmaped_dict

    def readdir(self, path: str, _fh: int = 0) -> Any:
        path_fixed = self.fix_path(path)
        if self.files.get(path) is not None:
            last_update = self.files[path].get("ftp_last_update")
            if  last_update is None:
                logger.warning(f"missing ftp_last_update for {path}")
                raise FuseOSError(errno.ENOENT)
            if time.time() - last_update < self.cache_duration:
                return self.get_cached_dir_list(path)
        logger.warning(f"cache miss for {path_fixed}")
        self.ftp.cmd_list([path_fixed])
        directory = self.ftp.list_result
        if directory is None or len(directory) == 0:
            return []
        ret = {}
        for item in directory:
            if item.name in [".", ".."]:
                continue
            if not item.is_dir:
                new_item = {"st_mode": (0o100444), "st_size": item.size_b}
            else:
                new_item = {
                    "st_mode": (0o46766),
                    "st_nlink": 2,  # Self-link and parent link
                    "st_size": 0,  # Size 0 for simplicity
                    # current timestamp for filebrowswer to be happy
                    "st_ctime": int(time.time()),  # Current time
                    "st_mtime": int(time.time()),  # Current time
                    "st_atime": int(time.time()),  # Current time
                    "ftp_last_update": 0,
                }
            ret[item.name] = new_item
            new_path = path if path.endswith("/") else path + "/"
            self.files[new_path + item.name] = new_item
        self.files[path] = {"st_mode": (0o46766), "st_size": 0, "ftp_last_update": time.time()}
        return ret

    def create(self, path: str, fi: int = 0) -> None:
        logger.info(f"Fuse: create {path}, fi={fi}")
        raise FuseOSError(errno.ENOENT)

    def mknod(self, path: str) -> None:
        logger.error(f"Fuse: lookup {path}")
        raise FuseOSError(errno.ENOENT)

    def write(self, path: str, _data: bytes, _offset: int, _fh: int) -> None:
        logger.error(f"Fuse: write {path}")
        raise FuseOSError(errno.EROFS)

    def mkdir(self, path: str, _mode: int) -> None:
        logger.error(f"Fuse: mkdir {path}")
        raise FuseOSError(errno.EROFS)

    def rmdir(self, path: str) -> None:
        logger.error(f"Fuse: rmdir {path}")
        raise FuseOSError(errno.EROFS)

    def unlink(self, path: str) -> None:
        logger.error(f"Fuse: unlink {path}")
        raise FuseOSError(errno.EROFS)

    def rename(self, old: str, new: str) -> None:
        logger.error(f"Fuse: rename {old} -> {new}")
        raise FuseOSError(errno.EROFS)

    def chmod(self, path: str, _mode: int) -> None:
        logger.error(f"Fuse: chmod {path}")
        raise FuseOSError(errno.EROFS)

    def chown(self, path: str, _uid: int, _gid: int) -> None:
        logger.error(f"Fuse: chown {path}")
        raise FuseOSError(errno.EROFS)

    def truncate(self, path: str) -> None:
        logger.info(f"Fuse: truncate {path}")
        raise FuseOSError(errno.EROFS)


if __name__ == "__main__":
    # logging.basicConfig(level=logging.DEBUG)
    parser = ArgumentParser(description="MAVLink FTP", formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        "-m",
        "--mavlink",
        default="udpin:127.0.0.1:14555",
        help="MAVLink connection specifier",
    )
    parser.add_argument(
        "--mountpoint",
        help="Path to the mountpoint",
    )

    args = parser.parse_args()

    if not os.path.exists(args.mountpoint):
        os.makedirs(args.mountpoint)

    fuse = FUSE(
        FTP(mavutil.mavlink_connection(args.mavlink)),
        args.mountpoint,
        foreground=True,
        ro=True,
        nothreads=True,
        allow_other=True,
    )
