import pytest
from unittest.mock import patch

from Tools.gittools.pre_commit_copyright import get_file_paths


def test_get_file_paths_no_args():
    """Test get_file_paths with no command line arguments."""
    with patch('sys.argv', ["pre_commit_copyright.py"]):
        with pytest.raises(SystemExit, match="2"):
            _ = get_file_paths()


@pytest.mark.parametrize(
    "argv, expected",
    [
        (
            ["pre_commit_copyright.py", "file1.py", "file2.py", "file3.py"],
            ["file1.py", "file2.py", "file3.py"]
        ),
        (
            ["pre_commit_copyright.py", "file1.py", "file2.py", "--ignore=file2.py"],
            ["file1.py"]
        ),
        (
            ["pre_commit_copyright.py", "file1.py", "file2.py", "file3.py", "--ignore=file1.py", "--ignore=file3.py"],
            ["file2.py"]
        ),
        (
            ["pre_commit_copyright.py", "file1.py", "file2.py", "--ignore=excluded_file.py", "--ignore=another_excluded.py"],
            ["file1.py", "file2.py"]
        ),
        (
            ["pre_commit_copyright.py", "file1.py", "--ignore=file1.py"],
            []
        ),
    ]
)
def test_get_file_paths_parametrized(argv, expected):
    """Test get_file_paths with various command line argument combinations."""
    with patch('sys.argv', argv):
        assert get_file_paths() == expected
