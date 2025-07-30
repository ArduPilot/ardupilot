#!/usr/bin/env python3

# flake8: noqa

"""
Downloads image and video files from an XFRobot camera connected via ethernet
"""

import os
import re
from argparse import ArgumentParser
from urllib.request import urlretrieve, urlopen
from urllib.error import URLError, HTTPError
from html.parser import HTMLParser

# prefix string for output
prefix_str = "xfrobot-download.py: "
ip_address_default = "192.168.144.108"

# directory suffixes
MEDIA_DIRS = {
    "image": "IMG",
    "video": "VID"
}

# HTML parser to extract links from HTML pages
class LinkExtractor(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []

    def handle_starttag(self, tag, attrs):
        if tag == 'a':
            for attr in attrs:
                if attr[0] == 'href':
                    self.links.append(attr[1])

# extract file links from HTML page
def extract_file_links(base_url):
    try:
        with urlopen(base_url) as response:
            html = response.read().decode('utf-8')
            parser = LinkExtractor()
            parser.feed(html)
            return [link for link in parser.links if re.search(r'\.(jpg|jpeg|png|mp4|mov)$', link, re.IGNORECASE)]
    except Exception as e:
        print(prefix_str + f"Failed to fetch or parse URL {base_url}: {e}")
        return []

# download files from the given list of links
# returns the number of successfully downloaded files
def download_files(base_url, links, dest_dir):
    count = 0
    for link in links:
        filename = os.path.basename(link)
        full_url = link if link.startswith("http") else base_url + filename
        dest_path = os.path.join(dest_dir, filename)
        print(prefix_str + f"Downloading {filename} from {full_url}")
        try:
            urlretrieve(full_url, dest_path)
            count += 1
        except (URLError, HTTPError) as e:
            print(prefix_str + f"Failed to download {filename}: {e}")
    return count

# main function
def main():
    parser = ArgumentParser(description="Download files from an XFRobot camera")
    parser.add_argument("--ipaddr", default=ip_address_default, help="IP address of camera")
    parser.add_argument("--dest", default=".", help="Destination directory")
    args = parser.parse_args()

    if not os.path.exists(args.dest):
        print(prefix_str + "Invalid destination directory")
        return

    for media_type, subdir in MEDIA_DIRS.items():
        print(prefix_str + f"Fetching {media_type} files")
        base_url = f"http://{args.ipaddr}/static/{subdir}/"
        links = extract_file_links(base_url)
        count = download_files(base_url, links, args.dest)
        print(prefix_str + f"Downloaded {count} {media_type} file(s)")

if __name__ == "__main__":
    main()
