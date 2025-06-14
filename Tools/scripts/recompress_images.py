#!/usr/bin/env python3
'''
script to re-compress images in a directory tree to be below a specified max file size.
useful for the hwdef directory where images can be large.
'''

import os
import sys
import argparse
from pathlib import Path
from PIL import Image
import io
from multiprocessing import Pool, cpu_count

def compress_image(input_path, output_path, max_size_bytes, image_format):
    quality = 95
    step = 5

    with Image.open(input_path) as img:
        img = img.convert("RGB")  # Ensure JPEG-compatible format
        while quality > 5:
            buffer = io.BytesIO()
            img.save(buffer, format=image_format, quality=quality, optimize=True)
            size = buffer.tell()
            if size <= max_size_bytes:
                with open(output_path, 'wb') as f:
                    f.write(buffer.getvalue())
                return size
            quality -= step
    return None

def update_readme_references(image_path, new_image_path):
    readme_path = image_path.parent / "README.md"
    if not readme_path.exists():
        return
    try:
        content = readme_path.read_text(errors="ignore")
        updated_content = content.replace(image_path.name, new_image_path.name)
        if content != updated_content:
            readme_path.write_text(updated_content)
            print(f"Updated README.md: {readme_path}")
    except Exception as e:
        print(f"Failed to update README.md: {e}")

def process_image(args):
    filepath, max_size_bytes = args
    ext = filepath.suffix.lower().lstrip(".")
    original_size = filepath.stat().st_size

    if original_size <= max_size_bytes:
        return

    print(f"Compressing: {filepath} ({original_size} bytes)")
    tmp_path = filepath.with_suffix(filepath.suffix + ".tmp")
    format = "JPEG" if ext in ("jpg", "jpeg") else "PNG"
    new_size = compress_image(filepath, tmp_path, max_size_bytes, format)

    if new_size:
        os.replace(tmp_path, filepath)
        print(f" -> Compressed to {new_size} bytes")
    elif ext == "png":
        # Try converting to JPEG
        jpg_path = filepath.with_suffix(".jpg")
        new_size = compress_image(filepath, jpg_path, max_size_bytes, "JPEG")
        if new_size:
            filepath.unlink(missing_ok=True)
            print(f" -> Converted PNG to JPEG: {jpg_path} ({new_size} bytes)")
            update_readme_references(filepath, jpg_path)
        else:
            print(f" -> Failed to convert/compress PNG to JPEG below {max_size_bytes} bytes")
            jpg_path.unlink(missing_ok=True)
        tmp_path.unlink(missing_ok=True)
    else:
        print(f" -> Failed to compress below {max_size_bytes} bytes")
        tmp_path.unlink(missing_ok=True)

def main():
    parser = argparse.ArgumentParser(description="Re-compress images to be below a max file size.")
    parser.add_argument("path", help="Path to search for images")
    parser.add_argument("--max-size", type=int, required=True, help="Max file size in bytes")
    parser.add_argument("--num-cpus", type=int, default=cpu_count(), help="Number of parallel workers")
    args = parser.parse_args()

    base_path = Path(args.path)
    if not base_path.exists():
        print(f"Error: path '{base_path}' does not exist.", file=sys.stderr)
        sys.exit(1)

    image_files = []
    for root, dirs, files in os.walk(base_path):
        for name in files:
            ext = name.lower().rsplit(".", 1)[-1]
            if ext in ("jpg", "jpeg", "png"):
                image_files.append((Path(root) / name, args.max_size))

    with Pool(processes=args.num_cpus) as pool:
        pool.map(process_image, image_files)

if __name__ == "__main__":
    main()
