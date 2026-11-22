"""Regression tests for adding bootloader sources to existing boards.

AP_FLAKE8_CLEAN
"""

import os
import subprocess
import tempfile
import unittest

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from unittest.mock import patch

from test_new_boards import TestNewBoards


class NewBoardChecksTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        original_directory = os.getcwd()
        os.chdir(self.directory.name)
        self.addCleanup(os.chdir, original_directory)
        self.hwdef_dir = Path('libraries/AP_HAL_ChibiOS/hwdef/testboard')
        self.hwdef_dir.mkdir(parents=True)
        self.main_hwdef = str(self.hwdef_dir / 'hwdef.dat')
        self.boot_hwdef = str(self.hwdef_dir / 'hwdef-bl.dat')
        Path(self.boot_hwdef).write_text('')
        self.binary = 'Tools/bootloaders/testboard_bl.bin'
        self.added = {self.boot_hwdef}

        self.checker = TestNewBoards.__new__(TestNewBoards)
        self.checker.progress = Mock()
        self.checker.get_added_files = lambda: self.added
        self.checker.get_added_or_modified_files = lambda: self.added
        self.checker.get_added_hwdef_paths = lambda: self.added & {self.main_hwdef, self.boot_hwdef}
        self.checker.run_git = Mock(return_value=self.binary + '\n')
        self.checker.is_odid_board = Mock(return_value=False)
        self.checker.build_board = Mock()
        self.checker.build_bootloader = Mock()
        self.board = SimpleNamespace(name='testboard', is_ap_periph=False,
                                     hal='ChibiOS', toolchain='arm-none-eabi')
        boards = patch('test_new_boards.board_list.BoardList', return_value=SimpleNamespace(boards=[self.board]))
        boards.start()
        self.addCleanup(boards.stop)

    def test_existing_board_and_binary_still_build_bootloader(self):
        Path(self.main_hwdef).write_text('')
        self.checker.run()
        self.checker.build_bootloader.assert_called_once_with(self.board)
        self.checker.build_board.assert_not_called()

    def test_existing_board_without_binary_is_rejected(self):
        Path(self.main_hwdef).write_text('')
        self.checker.run_git.return_value = ''
        with self.assertRaisesRegex(ValueError, 'requires its prebuilt bootloader binary'):
            self.checker.run()
        self.checker.build_bootloader.assert_not_called()

    def test_new_board_requires_readme(self):
        Path(self.main_hwdef).write_text('')
        self.added.add(self.main_hwdef)
        with self.assertRaisesRegex(ValueError, 'Missing README.md'):
            self.checker.run()

    def test_new_bootloader_only_board_requires_readme(self):
        with self.assertRaisesRegex(ValueError, 'Missing README.md'):
            self.checker.run()

    def test_new_board_requires_image(self):
        Path(self.main_hwdef).write_text('')
        self.added.add(self.main_hwdef)
        (self.hwdef_dir / 'README.md').write_text('# Test board\n')
        with self.assertRaisesRegex(ValueError, 'contains no local image references'):
            self.checker.run()

    def test_new_board_with_documentation_and_binary_builds_both(self):
        Path(self.main_hwdef).write_text('')
        (self.hwdef_dir / 'README.md').write_text('# Test board\n\n![Board](board.png)\n')
        (self.hwdef_dir / 'board.png').write_bytes(b'')
        self.added.update({self.main_hwdef, self.binary, str(self.hwdef_dir / 'board.png')})
        self.checker.run_git.return_value = ''
        self.checker.run()
        self.checker.build_board.assert_called_once_with(self.board)
        self.checker.build_bootloader.assert_called_once_with(self.board)

    def test_image_over_1MB_is_rejected(self):
        Path(self.main_hwdef).write_text('')
        (self.hwdef_dir / 'README.md').write_text('# Test board\n\n![Board](board.png)\n')
        image = self.hwdef_dir / 'board.png'
        image.write_bytes(b'\0' * (1024 * 1024 + 1))
        self.added.update({self.main_hwdef, self.binary, str(image)})
        with self.assertRaisesRegex(ValueError, 'must not exceed'):
            self.checker.run()
        self.checker.build_board.assert_not_called()

    def test_image_of_exactly_1MB_is_accepted(self):
        Path(self.main_hwdef).write_text('')
        (self.hwdef_dir / 'README.md').write_text('# Test board\n\n![Board](board.png)\n')
        image = self.hwdef_dir / 'board.png'
        image.write_bytes(b'\0' * (1024 * 1024))
        self.added.update({self.main_hwdef, self.binary, str(image)})
        self.checker.run_git.return_value = ''
        self.checker.run()
        self.checker.build_board.assert_called_once_with(self.board)


class ChangedImageFilesTest(unittest.TestCase):
    '''runs the image size check against git's real diff output'''

    HWDEF = 'libraries/AP_HAL_ChibiOS/hwdef'

    def git(self, *args):
        return subprocess.run(['git', *args], check=True, capture_output=True, text=True).stdout

    def write_image(self, path, content):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        original_directory = os.getcwd()
        os.chdir(self.directory.name)
        self.addCleanup(os.chdir, original_directory)
        # keep the user's git config and environment out of these tests;
        # tests that need a setting configure it in the test repository
        environ = patch.dict(os.environ, {'GIT_CONFIG_GLOBAL': os.devnull, 'GIT_CONFIG_NOSYSTEM': '1'})
        environ.start()
        self.addCleanup(environ.stop)
        for name in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_INDEX_FILE'):
            os.environ.pop(name, None)
        self.git('init', '-q', '-b', 'master')
        self.git('config', 'user.name', 'test')
        self.git('config', 'user.email', 'test@example.com')

        self.big = 2 * 1024 * 1024
        self.edited = os.urandom(self.big)
        self.write_image(f'{self.HWDEF}/old/edited.png', self.edited)
        self.write_image(f'{self.HWDEF}/old/moved.png', os.urandom(self.big))
        self.git('add', '.')
        self.git('commit', '-q', '-m', 'base')
        self.git('checkout', '-q', '-b', 'branch')

        self.checker = TestNewBoards.__new__(TestNewBoards)
        self.checker.master_branch = 'master'
        self.checker.tmpdir = None

    def commit(self):
        self.git('add', '-A')
        self.git('commit', '-q', '-m', 'change')

    def test_unchanged_rename_is_not_checked(self):
        os.renames(f'{self.HWDEF}/old/moved.png', f'{self.HWDEF}/new/moved.png')
        self.commit()
        self.assertIn('R100', self.git('diff', '--name-status', 'master'))
        changed = self.checker.get_added_or_modified_files()
        self.assertEqual(changed, set())
        self.checker.check_image_sizes(changed)

    def test_edited_rename_is_checked(self):
        os.remove(f'{self.HWDEF}/old/edited.png')
        self.write_image(f'{self.HWDEF}/new/edited.png', self.edited + b'\0' * 100000)
        self.commit()
        status = self.git('diff', '--name-status', 'master')
        self.assertRegex(status, r'^R0\d\d\t')
        changed = self.checker.get_added_or_modified_files()
        self.assertEqual(changed, {f'{self.HWDEF}/new/edited.png'})
        with self.assertRaisesRegex(ValueError, f'{self.HWDEF}/new/edited.png'):
            self.checker.check_image_sizes(changed)

    def test_non_ascii_path_is_checked(self):
        self.write_image(f'{self.HWDEF}/new/board \u00e9.png', os.urandom(self.big))
        self.commit()
        self.assertIn(f'"{self.HWDEF}/new/board \\303\\251.png"', self.git('diff', '--name-status', 'master'))
        changed = self.checker.get_added_or_modified_files()
        self.assertEqual(changed, {f'{self.HWDEF}/new/board \u00e9.png'})
        with self.assertRaisesRegex(ValueError, 'board \u00e9.png'):
            self.checker.check_image_sizes(changed)

    def test_non_ascii_path_with_quotepath_false(self):
        self.git('config', 'core.quotePath', 'false')
        self.write_image(f'{self.HWDEF}/new/board-\u00e9.png', os.urandom(self.big))
        self.commit()
        self.assertEqual(self.checker.get_added_or_modified_files(), {f'{self.HWDEF}/new/board-\u00e9.png'})

    def test_unchanged_rename_with_renames_false(self):
        self.git('config', 'diff.renames', 'false')
        os.renames(f'{self.HWDEF}/old/moved.png', f'{self.HWDEF}/new/moved.png')
        self.commit()
        self.assertEqual(self.checker.get_added_or_modified_files(), set())

    def test_copy_with_renames_copies(self):
        self.git('config', 'diff.renames', 'copies')
        self.write_image(f'{self.HWDEF}/new/copy.png', self.edited)
        self.write_image(f'{self.HWDEF}/old/edited.png', self.edited + b'\0' * 1000)
        self.commit()
        self.assertEqual(self.checker.get_added_or_modified_files(),
                         {f'{self.HWDEF}/new/copy.png', f'{self.HWDEF}/old/edited.png'})


if __name__ == '__main__':
    unittest.main()
