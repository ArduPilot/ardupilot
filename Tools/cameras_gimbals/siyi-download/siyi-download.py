#!/usr/bin/env python3

"""
Downloads files from a Siyi camera connected via ethernet

AP_FLAKE8_CLEAN
"""

#
# utility to download files from a siyi camera
#

import os
import json
from argparse import ArgumentParser
from urllib.request import urlopen, urlretrieve
from urllib.parse import urlencode
from urllib.error import URLError, HTTPError
from enum import Enum

# prefix string for text outout to user
prefix_str = "siyi-download.py: "
ip_address_default = "192.168.144.25"


# media types
class MediaTypes(Enum):
    IMAGE = 0
    VIDEO = 1


# media type strings. used to display to user
MEDIA_TYPE_STR = ["image", "video"]


# get URL for list of directories
def get_dirlist_url(ip_address, media_type):
    params = {'media_type': media_type}
    return f"http://{ip_address}:82/cgi-bin/media.cgi/api/v1/getdirectories?" + urlencode(params)


# get URL for list of files in a directory
def get_filelist_url(ip_address, media_type, dir_path):
    params = {
        'media_type': str(media_type),
        'path': dir_path,
        'start': 0,
        'count': 999
    }
    return f"http://{ip_address}:82/cgi-bin/media.cgi/api/v1/getmedialist?" + urlencode(params)


# download files from camera
def download_files(ip_address, dest_dir):

    # repeat for images and videos
    for media_type in [mt.value for mt in MediaTypes]:

        # display output to user
        print(prefix_str + f"downloading {MEDIA_TYPE_STR[media_type]} files")

        # download list of directories in JSON format
        dir_list_url = get_dirlist_url(ip_address, media_type)
        with urlopen(dir_list_url) as get_dir_url:
            dir_dict = json.load(get_dir_url)

            # check that the request succeeded
            if (not dir_dict['success']):
                exit(prefix_str + "failed to get list of directories")

            # check response includes 'data'
            if ('data' not in dir_dict.keys()):
                exit(prefix_str + "could not get list of directories, no 'data' in response")
            dir_dict_data = dir_dict['data']

            # check response includes 'directories'
            if ('directories' not in dir_dict_data.keys()):
                exit(prefix_str + "could not get list of directories, no 'directories' in response")
            dir_dict_data_directories = dir_dict_data['directories']

            # create list of directories from 'path' values
            dir_list = []
            for dir in dir_dict_data_directories:
                if ('path' in dir.keys()):
                    dir_list.append(dir['path'])
            print(prefix_str + f"{len(dir_list)} directories")

            # get list of files in each directory
            for dir_path in dir_list:
                filenames_url = get_filelist_url(ip_address, media_type, dir_path)
                with urlopen(filenames_url) as get_filenames_url:
                    filename_dict = json.load(get_filenames_url)

                    # check that the request succeeded
                    if (not filename_dict['success']):
                        exit(prefix_str + "failed to get list of files")

                    # check response includes 'data'
                    if ('data' not in filename_dict.keys()):
                        exit(prefix_str + "could not get list of files, no 'data' in response")
                    filename_dict_data = filename_dict['data']

                    # check response includes 'list'
                    if ('list' not in filename_dict_data.keys()):
                        exit(prefix_str + "could not get list of files, no 'list' in response")
                    filename_dict_data_list = filename_dict_data['list']
                    print(prefix_str + f"{len(filename_dict_data_list)} files")

                    # download each image
                    for fileinfo in filename_dict_data_list:
                        if ('name' not in fileinfo.keys() or 'url' not in fileinfo.keys()):
                            exit(prefix_str + "could not get list of files, no 'name' or 'url' in response")
                        filename = fileinfo['name']
                        file_url = fileinfo['url']

                        # correct incorrect ip address in returned url
                        file_url_fixed = file_url.replace(ip_address_default, ip_address)

                        # download file
                        print(prefix_str + f"downloading {filename} from {file_url_fixed}")
                        dest_filename = os.path.join(dest_dir, filename)
                        try:
                            urlretrieve(file_url_fixed, dest_filename)
                        except (URLError, HTTPError) as e:
                            print(prefix_str + f"failed to download {filename}: {e}")


# main function
def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--ipaddr", default=ip_address_default, help="IP address of camera")
    parser.add_argument("--dest", default=".", help="destination directory where downloaded files will be saved")
    args = parser.parse_args()

    # check destination directory exists
    if not os.path.exists(args.dest):
        exit(prefix_str + "invalid destination directory")

    # download files
    download_files(args.ipaddr, args.dest)


# main
if __name__ == "__main__":
    main()
