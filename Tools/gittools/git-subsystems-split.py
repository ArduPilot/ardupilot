#!/usr/bin/env python3

import argparse
import os
import subprocess
import sys
from pathlib import Path
import shutil
import tempfile


def usage(file=sys.stdout):
    print("""\
git subsystems-split [OPTIONS]

Ardupilot's git extension.

Split HEAD commit into commits separated by subsystems (vehicles, libraries and
folders in the project's root). Basically, reset and call commit-subsystems.

If neither --copy or --edit is passed, then subsystems-split will try to make
the original commit's message into a template for commit-subsystems.

Options:
    --copy
    Make all commits have exactly the same message as the HEAD commit.

    --edit
    Edit the commit message as a template for commit-subsystems.
""", file=file)


def get_git_output(*args):
    return subprocess.check_output(['git'] + list(args), text=True).strip()


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--copy', action='store_true')
    parser.add_argument('--edit', action='store_true')
    parser.add_argument('-h', '--help', action='store_true')
    args, unknown = parser.parse_known_args()

    if args.help:
        usage()
        sys.exit(0)

    if unknown:
        usage(sys.stderr)
        sys.exit(1)

    if args.copy and args.edit:
        print("Options --copy and --edit can't be combined.", file=sys.stderr)
        sys.exit(1)

    script_dir = Path(__file__).resolve().parent
    git_dir = Path(get_git_output('rev-parse', '--git-dir'))
    # git_root = Path(get_git_output('rev-parse', '--show-toplevel'))
    msg_file = git_dir / "SUBSYSTEMS_SPLIT_MSG"

    # Extract author and commit message
    author_name = get_git_output('log', '-n', '1', '--format=%an')
    author_email = get_git_output('log', '-n', '1', '--format=%ae')
    author = f"{author_name} <{author_email}>"

    msg = get_git_output('log', '-n', '1', '--format=%B')
    msg_file.write_text(msg)

    if args.edit:
        editor = os.getenv('EDITOR')
        if not editor:
            print("Environment variable EDITOR is required for option --edit.", file=sys.stderr)
            sys.exit(1)
        subprocess.run([editor, str(msg_file)])
    elif not args.copy:
        lines = msg_file.read_text().splitlines()
        if lines and any(c == ':' for c in lines[0].split(None, 1)[0]):
            # Message looks like "subsystem: text"
            lines[0] = f"$subsystem: {lines[0].split(':', 1)[1].lstrip()}"
            msg_file.write_text("\n".join(lines) + "\n")
        else:
            with tempfile.NamedTemporaryFile('w', delete=False) as buff:
                if lines:
                    first_line = lines[0].lstrip()
                    new_first_line = f"$subsystem: {first_line[0].lower()}{first_line[1:]}"
                    buff.write(new_first_line + '\n')
                if len(lines) > 1:
                    buff.write("\n".join(lines[1:]) + '\n')
                buff_name = buff.name
            shutil.move(buff_name, msg_file)

    head = get_git_output('rev-parse', 'HEAD')
    subprocess.run(['git', 'reset', f'{head}~1', '--soft'], check=True)

    try:
        subprocess.run([
            str(script_dir / 'git-commit-subsystems'),
            '-F', str(msg_file),
            f'--author={author}'
        ], check=True)
    except subprocess.CalledProcessError:
        print("Error on calling git-commit-subsystems.", file=sys.stderr)
        subprocess.run(['git', 'reset', head], check=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
