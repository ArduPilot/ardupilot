# AP_FLAKE8_CLEAN
"""Strict standard-library template substitution; no expressions or custom DSL."""

from pathlib import Path
from string import Template


class Templates:
    def __init__(self):
        directory = Path(__file__).with_name('templates')
        self.templates = {path.name: Template(path.read_text(encoding='utf-8')) for path in directory.glob('*.tmpl')}

    def render(self, template, **values):
        return self.templates[template + '.tmpl'].substitute(values)


def guarded(text, dependency):
    if dependency is None:
        return text
    return f'#if {dependency}\n{text}#endif // {dependency}\n'
