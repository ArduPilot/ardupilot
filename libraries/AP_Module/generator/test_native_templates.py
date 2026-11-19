# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Template substitution is strict, deterministic and never recursive."""

import json
import runpy
import shutil

from pathlib import Path
from types import SimpleNamespace

import generate
import native_templates
import pytest

from native_templates import Templates


def test_nested_templates(tmp_path):
    directory = tmp_path / 'rust'
    directory.mkdir()
    (directory / 'method.rs.tmpl').write_text('fn ${name}() {\n${body}}\n', encoding='utf-8')
    templates = Templates(tmp_path)
    expected = 'fn get_gyro() {\n    todo!();\n}\n'
    assert templates.render('rust/method.rs', name='get_gyro', body='    todo!();\n') == expected
    assert templates.render('rust/method.rs', name='get_roll', body='') == 'fn get_roll() {\n}\n'
    assert templates.render('rust/method.rs', name='get_gyro', body='    todo!();\n') == expected


def test_literal_dollars_and_nonrecursive_values(tmp_path):
    (tmp_path / 'macro.rs.tmpl').write_text('$$crate::call!($$x, ${value});\n', encoding='utf-8')
    templates = Templates(tmp_path)
    assert templates.render('macro.rs', value='$not_a_placeholder') == '$crate::call!($x, $not_a_placeholder);\n'


def test_missing_template(tmp_path):
    with pytest.raises(ValueError, match='missing native binding template: rust/missing.rs.tmpl'):
        Templates(tmp_path).render('rust/missing.rs')


def test_missing_placeholder(tmp_path):
    (tmp_path / 'call.cpp.tmpl').write_text('${receiver}->${method}();', encoding='utf-8')
    with pytest.raises(ValueError, match="call.cpp.tmpl:.*'method'"):
        Templates(tmp_path).render('call.cpp', receiver='ahrs')


def test_unused_substitution(tmp_path):
    (tmp_path / 'value.rs.tmpl').write_text('${name}', encoding='utf-8')
    with pytest.raises(ValueError, match='value.rs.tmpl: unused substitutions: typo'):
        Templates(tmp_path).render('value.rs', name='gyro', typo='unused')


@pytest.mark.parametrize('text,expected', [
    ('', []), ('line', ['line']), ('line\n', ['line']), ('line\n\n', ['line', '']),
    ('first\rsecond\u0085third\u2028fourth\u2029fifth\n', ['first\rsecond\u0085third\u2028fourth\u2029fifth']),
])
def test_render_lines_preserves_non_lf_characters(tmp_path, monkeypatch, text, expected):
    (tmp_path / 'lines.tmpl').write_text('${text}', encoding='utf-8')
    monkeypatch.setattr(native_templates, '_templates', Templates(tmp_path))
    assert native_templates.render_lines('lines', text=text) == expected


@pytest.mark.parametrize('content', ['$!', '${unterminated', '$', '${invalid.name}'])
def test_invalid_placeholder(tmp_path, content):
    (tmp_path / 'invalid.tmpl').write_text(content, encoding='utf-8')
    with pytest.raises(ValueError, match='invalid.tmpl: Invalid placeholder'):
        Templates(tmp_path).render('invalid')


def test_template_change_regenerates_only_its_output(tmp_path, monkeypatch):
    directory = tmp_path / 'templates'
    shutil.copytree(Path(native_templates.__file__).with_name('templates'), directory)
    spec = json.loads((generate.ROOT / 'generator/host_api.json').read_text())
    monkeypatch.setattr(native_templates, '_templates', Templates(directory))
    before = generate.generate(spec)
    output = tmp_path / 'generated'
    generate.write_outputs(before, output)
    template = directory / 'rust/host_api.rs.tmpl'
    template.write_text(template.read_text() + '// template edit\n', encoding='utf-8')
    monkeypatch.setattr(native_templates, '_templates', Templates(directory))
    after = generate.generate(spec)
    changed = {name for name in before if before[name] != after[name]}
    assert changed == {'sdk/rust/src/api.rs'}
    with pytest.raises(SystemExit, match='stale generated file:.*sdk/rust/src/api.rs'):
        generate.write_outputs(after, output, check=True)
    generate.write_outputs(after, output)
    generate.write_outputs(after, output, check=True)
    template.unlink()
    monkeypatch.setattr(native_templates, '_templates', Templates(directory))
    with pytest.raises(ValueError, match='missing native binding template: rust/host_api.rs.tmpl'):
        generate.generate(spec)


def waf_task(root, output, monkeypatch):
    monkeypatch.syspath_prepend(str(generate.ROOT.parents[1] / 'modules/waf'))
    from waflib import Context
    context = Context.Context()
    context.srcnode = context.root.make_node(str(root))
    context.bldnode = context.root.make_node(str(output))
    tasks = []

    class Build:
        srcnode = context.srcnode
        bldnode = context.bldnode

        def __call__(self, **task):
            tasks.append(SimpleNamespace(**task))

    runpy.run_path(str(generate.ROOT / 'wscript'))['build'](Build())
    return tasks[0]


def test_waf_tracks_templates_and_declares_generated_outputs(tmp_path, monkeypatch):
    root = generate.ROOT.parents[1]
    task = waf_task(root, tmp_path / 'build', monkeypatch)
    inputs = {Path(node.abspath()) for node in task.source}
    templates = set((generate.ROOT / 'generator/templates').rglob('*.tmpl'))
    assert templates
    assert templates <= inputs
    outputs = {node.path_from(task.output_dir) for node in task.target}
    spec = json.loads((generate.ROOT / 'generator/host_api.json').read_text())
    assert outputs == set(generate.generate(spec))


def test_waf_tracks_template_addition_and_removal(tmp_path, monkeypatch):
    directory = tmp_path / 'source/libraries/AP_Module/generator/templates/rust/nested'
    directory.mkdir(parents=True)
    (tmp_path / 'source/libraries/AP_Scripting/generator').mkdir(parents=True)
    template = directory / 'extra.rs.tmpl'
    template.write_text('initial\n', encoding='utf-8')

    def tracked():
        task = waf_task(tmp_path / 'source', tmp_path / 'build', monkeypatch)
        return {Path(node.abspath()) for node in task.source}

    assert template in tracked()
    template.unlink()
    assert template not in tracked()
    template.write_text('replacement\n', encoding='utf-8')
    assert template in tracked()
