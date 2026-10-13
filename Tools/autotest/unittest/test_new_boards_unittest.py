"""Regression tests for adding bootloader sources to existing boards.

AP_FLAKE8_CLEAN
"""

import os
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
        self.added.update({self.main_hwdef, self.binary, str(self.hwdef_dir / 'board.png')})
        self.checker.run_git.return_value = ''
        self.checker.run()
        self.checker.build_board.assert_called_once_with(self.board)
        self.checker.build_bootloader.assert_called_once_with(self.board)


if __name__ == '__main__':
    unittest.main()
