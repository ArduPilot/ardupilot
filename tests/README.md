> [!CAUTION]
> `pytest tests/` will be run in GitHub Actions.

All Python files in this directory that meet [pytest discovery rules](https://docs.pytest.org/en/stable/explanation/goodpractices.html#conventions-for-python-test-discovery) will be examined by pytest and all tests that it finds will be run.

Files of the form `test_*.py` or `*_test.py` should avoid code at global scope because it will be executed in pytest's discovery process.  Classes, functions, and code in `if __name__ == "__main__":` blocks will not be run in this discovery process, but code at global scope (including imports) will.