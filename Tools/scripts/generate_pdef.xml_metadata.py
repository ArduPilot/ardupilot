#!/usr/bin/env python3

'''
Legacy/backfill publisher for apm.pdef.xml files from historical ArduPilot releases.

Normal firmware builds generate parameter metadata through build_binaries.py.
This script rebuilds metadata for historical Git tags. Uploading the generated
files requires an explicit --upload option.

AP_FLAKE8_CLEAN

SPDX-FileCopyrightText: 2024 Amilcar do Carmo Lucas <amilcar.lucas@iav.de>

SPDX-License-Identifier: GPL-3.0-or-later
'''

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


VEHICLE_TYPES = ["Copter", "Plane", "Rover", "ArduSub", "Tracker"]
RSYNC_USERNAME = 'amilcar'
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_REPOSITORY = SCRIPT_DIR.parent.parent
VERSION_RE = re.compile(r'^(?P<vehicle>.+)-(?P<version>\d+\.\d+\.\d+)$')


def version_key(version: str):
    '''Return a comparable semantic version tuple, rejecting invalid input.'''
    try:
        parts = tuple(int(part) for part in version.split('.'))
    except ValueError as error:
        raise argparse.ArgumentTypeError(f'Invalid version: {version}') from error
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f'Invalid version: {version}')
    return parts


def get_vehicle_tags(repository: Path, vehicle_type: str):
    '''Return release tags and versions for a vehicle, ordered by version.'''
    tags_output = subprocess.check_output(
        ['git', '-C', repository, 'tag', '--list', f'{vehicle_type}-*'],
        text=True,
    )
    releases = []
    for tag in tags_output.splitlines():
        match = VERSION_RE.fullmatch(tag)
        if match is None or match.group('vehicle') != vehicle_type:
            continue
        version = match.group('version')
        releases.append((version_key(version), tag, version))
    return [(tag, version) for _, tag, version in sorted(releases)]


def add_provenance_comment(xml_path: Path, git_tag: str, git_sha: str):
    '''Add deterministic provenance for parsers without the corresponding options.'''
    contents = xml_path.read_text(encoding='utf-8')
    comment = f'<!-- Generated from git tag {git_tag} ({git_sha}) -->\n'
    xml_path.write_text(contents.replace('\n', '\n' + comment, 1), encoding='utf-8')


def publish_file(source: Path, destination: Path):
    '''Atomically publish a generated metadata file into its destination directory.'''
    destination.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary_name = tempfile.mkstemp(prefix='.apm.pdef-', dir=destination.parent)
    try:
        with os.fdopen(fd, 'wb') as temporary_file:
            with source.open('rb') as source_file:
                shutil.copyfileobj(source_file, temporary_file)
        os.replace(temporary_name, destination)
    except Exception:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)
        raise


def create_one_pdef_xml_file(repository: Path, vehicle_type: str, destination: Path, git_tag: str):
    '''Generate and atomically publish metadata from an isolated tag checkout.'''
    with tempfile.TemporaryDirectory(prefix='pdef-metadata-') as temporary_dir:
        worktree = Path(temporary_dir) / 'source'
        subprocess.run(
            ['git', '-C', repository, 'worktree', 'add', '--detach', worktree, git_tag],
            check=True,
        )
        try:
            git_sha = subprocess.check_output(
                ['git', '-C', worktree, 'rev-parse', 'HEAD'], text=True,
            ).strip()
            parser_path = worktree / 'Tools' / 'autotest' / 'param_metadata' / 'param_parse.py'
            help_output = subprocess.check_output(
                [sys.executable, parser_path, '--help'],
                text=True,
                stderr=subprocess.STDOUT,
                cwd=worktree,
            )
            supports_git_sha = '--git-sha' in help_output
            supports_git_tag = '--git-tag' in help_output
            command = [sys.executable, parser_path, '--vehicle', vehicle_type, '--format', 'xml']
            if supports_git_sha:
                command.extend(['--git-sha', git_sha])
            if supports_git_tag:
                command.extend(['--git-tag', git_tag])
            subprocess.run(command, check=True, cwd=worktree)

            metadata_path = worktree / 'apm.pdef.xml'
            if not metadata_path.is_file():
                raise FileNotFoundError(f'Parameter metadata was not generated: {metadata_path}')
            if not (supports_git_sha and supports_git_tag):
                add_provenance_comment(metadata_path, git_tag, git_sha)
            publish_file(metadata_path, destination / 'apm.pdef.xml')
        finally:
            subprocess.run(
                ['git', '-C', repository, 'worktree', 'remove', '--force', worktree],
                check=False,
            )


def sync_to_remote(vehicle_dir: Path, password_file: Path):
    '''Upload one vehicle's generated metadata after explicit user opt-in.'''
    rsync_command = [
        'rsync',
        '-avz',
        '--progress',
        f'--password-file={password_file}',
        f'{vehicle_dir}/',
        f'{RSYNC_USERNAME}@firmware.ardupilot.org::param_versioned/{vehicle_dir.name}/',
    ]
    print(f'Synchronizing {vehicle_dir}...')
    subprocess.run(rsync_command, check=True)


def main():
    parser = argparse.ArgumentParser(description='Generate historical apm.pdef.xml metadata.')
    parser.add_argument('--repository', type=Path, default=DEFAULT_REPOSITORY,
                        help='ArduPilot repository to read (default: this script\'s repository)')
    parser.add_argument('--output-dir', type=Path, default=Path.cwd(),
                        help='Directory in which vehicle release directories are generated')
    parser.add_argument('--vehicle', choices=VEHICLE_TYPES, action='append',
                        help='Vehicle to backfill; may be repeated (default: all)')
    parser.add_argument('--start-with', type=version_key, metavar='X.Y.Z',
                        help='Generate only releases with version X.Y.Z or newer')
    parser.add_argument('--upload', action='store_true',
                        help='Upload generated files using rsync after generation')
    parser.add_argument('--rsync-password-file', type=Path, default=Path.cwd() / '.rsync_pass',
                        help='Password file used with --upload')
    args = parser.parse_args()

    repository = args.repository.resolve()
    if not (repository / '.git').exists():
        parser.error(f'Not an ArduPilot repository: {repository}')
    output_dir = args.output_dir.resolve()
    vehicle_types = args.vehicle if args.vehicle is not None else VEHICLE_TYPES
    if args.upload and not args.rsync_password_file.is_file():
        parser.error(f'Rsync password file not found: {args.rsync_password_file}')

    for vehicle_type in vehicle_types:
        vehicle_dir = 'Sub' if vehicle_type == 'ArduSub' else vehicle_type
        destination_vehicle_dir = output_dir / vehicle_dir
        for git_tag, version in get_vehicle_tags(repository, vehicle_type):
            release_version = version_key(version)
            if args.start_with is not None and release_version < args.start_with:
                continue
            major, minor, _ = release_version
            if major == 3 and vehicle_type != 'AP_Periph':
                continue
            if major == 4 and minor == 0 and vehicle_type != 'ArduSub':
                continue
            create_one_pdef_xml_file(
                repository,
                vehicle_type,
                destination_vehicle_dir / f'stable-{version}',
                git_tag,
            )
        if args.upload:
            sync_to_remote(destination_vehicle_dir, args.rsync_password_file.resolve())


if __name__ == '__main__':
    main()
