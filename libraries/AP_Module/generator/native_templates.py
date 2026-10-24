# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Strict build-time source templates; no expressions or runtime codecs."""

from pathlib import Path
from string import Template


class Templates:
    def __init__(self, directory=None):
        directory = Path(directory) if directory is not None else Path(__file__).with_name('templates')
        self.templates = {
            path.relative_to(directory).as_posix(): Template(path.read_text(encoding='utf-8'))
            for path in sorted(directory.rglob('*.tmpl'))
        }

    def render(self, template_name, **values):
        filename = template_name + '.tmpl'
        try:
            template = self.templates[filename]
        except KeyError:
            raise ValueError(f'missing native binding template: {filename}') from None
        try:
            text = template.substitute(values)
            identifiers = {match.group('named') or match.group('braced')
                           for match in template.pattern.finditer(template.template)}
            unused = values.keys() - identifiers
            if unused:
                raise ValueError('unused substitutions: ' + ', '.join(sorted(unused)))
            return text
        except (KeyError, ValueError) as error:
            raise ValueError(f'native binding template {filename}: {error}') from error


_templates = Templates()


def render(template_name, **values):
    return _templates.render(template_name, **values)


def render_lines(template_name, **values):
    """Joinable LF-delimited lines, preserving other characters in source text."""
    lines = render(template_name, **values).split('\n')
    return lines[:-1] if lines[-1] == '' else lines
