#!/usr/bin/env python3
"""Regression tests for conservative stack call graph refinements.

AP_FLAKE8_CLEAN
"""

import importlib.util
import os
import re
import shutil
import subprocess
import tempfile
import unittest

from contextlib import ExitStack
from contextlib import redirect_stdout
from io import StringIO
from unittest.mock import patch

import stack_analysis as sa


class GraphFixture:
    def __init__(self, root):
        self.root = root
        self.p = sa.Program('unused')
        self.classes = sa.ClassModel()
        self.lines = []

    def func(self, name, size=8, dynamic=False):
        f = sa.Func(name, sa.short_name(name), 'fixture', size, dynamic, 'fixture', 'ci')
        f.names = {f.name}
        self.p.funcs[name] = f
        self.p.by_symbol[name] = [f]
        self.p.demangled[name] = name
        return f

    def edge(self, caller, callee, source=None, missing=False):
        if source is None:
            loc = 'elf'
        else:
            self.lines.append(source)
            col = source.index('(') + 1
            loc = 'missing.cpp:1:%u' % col if missing else 'fixture.cpp:%u:%u' % (len(self.lines), col)
        caller.edges.append((callee if isinstance(callee, str) else callee.title, loc))

    def analyser(self, rules=(), call_limits=()):
        with open(os.path.join(self.root, 'fixture.cpp'), 'w') as f:
            f.write('\n'.join(self.lines) + '\n')
        self.classes.index()
        return sa.Analyser(self.p, self.classes, self.root, self.root, [], 'none', rules, call_limits)

    def annotation(self, f, value, block=False):
        if block:
            self.lines.extend(['/*', ' * @StackCallLimit: %s' % value, ' * Bound justified by the caller.', ' */'])
        else:
            self.lines.append('// @StackCallLimit: %s' % value)
        self.lines.append('void %s {}' % f.title)
        f.loc = 'fixture.cpp:%u:1' % len(self.lines)


class StackAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.g = GraphFixture(self.temp.name)

    def telemetry(self, call='run_wfq_scheduler()', missing=False, packet_call='process_packet()', derived=False):
        g = self.g
        g.classes.bases = {'AP_RCTelemetry': [], 'AP_Spektrum_Telem': ['AP_RCTelemetry'],
                           'AP_CRSF_Telem': ['AP_RCTelemetry'],
                           'AP_Frsky_SPort_Passthrough': ['OtherBase', 'AP_RCTelemetry'], 'OtherBase': []}
        spektrum = g.func('AP_Spektrum_Telem::_get_telem_data()', 10)
        crsf = g.func('AP_CRSF_Telem::_get_telem_data()', 12)
        scheduler = g.func('AP_RCTelemetry::run_wfq_scheduler()', 20)
        sp_packet = g.func('AP_Spektrum_Telem::process_packet()', 30)
        cr_packet = g.func('AP_CRSF_Telem::process_packet()', 100)
        fr_packet = g.func('AP_Frsky_SPort_Passthrough::process_packet()', 40)
        frsky = g.func('AP_Frsky_SPort_Passthrough::get_telem_data()', 50)
        g.classes.slots = {'AP_RCTelemetry': {0: cr_packet.title},
                           'AP_Spektrum_Telem': {0: sp_packet.title},
                           'AP_CRSF_Telem': {0: cr_packet.title},
                           'AP_Frsky_SPort_Passthrough': {0: fr_packet.title}}
        if derived:
            g.classes.bases['SpektrumDerived'] = ['AP_Spektrum_Telem']
            override = g.func('SpektrumDerived::process_packet()', 250)
            g.classes.slots['SpektrumDerived'] = {0: override.title}
        g.p.poly_by_func[('fixture', scheduler.title)] = {('AP_RCTelemetry', 0)}
        g.edge(spektrum, scheduler, call, missing=missing)
        g.edge(crsf, scheduler, 'run_wfq_scheduler()')
        g.edge(scheduler, sa.INDIRECT, packet_call)
        g.edge(cr_packet, frsky, 'other->get_telem_data()')
        g.edge(frsky, scheduler, 'run_wfq_scheduler()')
        return g.analyser(), spektrum, crsf, scheduler

    def test_same_telemetry_receiver_and_nested_frsky(self):
        a, spektrum, crsf, generic = self.telemetry()
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(spektrum)[0], 60)
        depth, path = v.worst(crsf)
        self.assertEqual(depth, 242)
        self.assertEqual([f.name for f in path if isinstance(f, sa.Func)],
                         ['_get_telem_data', 'run_wfq_scheduler', 'process_packet',
                          'get_telem_data', 'run_wfq_scheduler', 'process_packet'])
        self.assertFalse(v.reachable([spektrum, crsf])[2])
        self.assertGreater(v.worst(generic)[0], 60)

    def test_explicit_this_and_qualified_base(self):
        for call in ('this->run_wfq_scheduler()', 'AP_RCTelemetry::run_wfq_scheduler()',
                     'this->AP_RCTelemetry::run_wfq_scheduler()'):
            with self.subTest(call=call):
                self.g = GraphFixture(self.temp.name)
                a, spektrum, _, _ = self.telemetry(call)
                self.assertEqual(a.variant('rcin', 'rcin').worst(spektrum)[0], 60)

    def test_other_receiver_stays_conservative(self):
        for call in ('other->run_wfq_scheduler()', 'other.run_wfq_scheduler()',
                     'other->AP_RCTelemetry::run_wfq_scheduler()', 'other.AP_RCTelemetry::run_wfq_scheduler()'):
            with self.subTest(call=call):
                self.g = GraphFixture(self.temp.name)
                a, spektrum, _, generic = self.telemetry(call)
                v = a.variant('rcin', 'rcin')
                self.assertEqual(v.worst(spektrum)[0], spektrum.size + v.worst(generic)[0])

    def test_missing_source_stays_conservative(self):
        a, spektrum, _, generic = self.telemetry(missing=True)
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(spektrum)[0], spektrum.size + v.worst(generic)[0])

    def test_virtual_call_on_other_object_in_shared_method_stays_broad(self):
        a, spektrum, _, generic = self.telemetry(packet_call='other->process_packet()')
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(spektrum)[0], spektrum.size + v.worst(generic)[0])

    def test_override_in_subclass_of_known_receiver_remains_reachable(self):
        a, spektrum, _, _ = self.telemetry(derived=True)
        self.assertEqual(a.variant('rcin', 'rcin').worst(spektrum)[0], 280)

    def message_graph(self, leaf=False, nested=False):
        g = self.g
        msg = g.func('MSG()', 10)
        critical = g.func('Critical()', 20)
        ensure = g.func('Ensure()', 30)
        emit = g.func('Emit()', 100)
        dynamic = g.func('Dynamic()', 40, dynamic=True)
        tail = g.func('Tail()', 200)
        g.edge(msg, critical)
        g.edge(critical, ensure)
        g.edge(ensure, emit)
        g.edge(emit, tail)
        g.edge(dynamic, critical)
        if nested:
            g.edge(critical, dynamic)
        rules = [((True, {'main'}), r'^Ensure\(', r'^Emit\(', leaf, r'^MSG\(', r'^(Critical|Ensure)\(')]
        return g.analyser(rules), msg, dynamic, emit

    def test_contextual_suppression_keeps_main_and_dynamic_formats(self):
        a, msg, dynamic, _ = self.message_graph()
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(msg)[0], 60)
        self.assertEqual(v.worst(dynamic)[0], 390)
        self.assertEqual(a.variant('main', 'main').worst(msg)[0], 360)
        self.assertTrue(v.reachable([dynamic])[1])

    def test_leaving_message_chain_restores_dynamic_format(self):
        a, msg, _, _ = self.message_graph(nested=True)
        v = a.variant('rcin', 'rcin')
        self.assertGreaterEqual(v.worst(msg)[0], 400)

    def test_contextual_leaf_keeps_frame_but_not_descendants(self):
        a, msg, dynamic, _ = self.message_graph(leaf=True)
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(msg)[0], 160)
        self.assertEqual(v.worst(dynamic)[0], 390)

    def test_contextual_suppression_starts_at_inlined_ancestor(self):
        g = self.g
        outer = g.func('Outer()', 10)
        helper = g.func('Helper()', 20)
        target = g.func('Target()', 100)
        g.edge(outer, helper, 'Helper()')
        loc = 'fixture.cpp:1:7'
        g.p.inline_sites[(outer.title, loc)] = {('Via()', outer.title)}
        g.edge(helper, target)
        rules = [(None, r'^Helper\(', r'^Target\(', False, r'^Via\(', r'^Helper\(')]
        v = g.analyser(rules).variant('main', 'main')
        self.assertEqual(v.worst(outer)[0], 30)
        self.assertEqual(v.worst(helper)[0], 120)

    def test_singleton_condition_follows_helpers_only_from_worker(self):
        g = self.g
        worker = g.func('Worker()', 10)
        helper = g.func('Helper()', 20)
        lookup = g.func('Lookup()', 30)
        init = g.func('Init()', 100)
        g.edge(worker, helper)
        g.edge(helper, lookup)
        g.edge(lookup, init)
        a = g.analyser([(None, '^Lookup', '^Init', False, '^Worker', None)])
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(worker)[0], 60)
        self.assertEqual(v.worst(lookup)[0], 130)

    def test_statustext_guard_requires_every_linked_predicate_to_be_true(self):
        for returns, expected in (([], 108), ([1], 8), ([None], 108), ([1, 0], 108), ([1, None], 108)):
            with self.subTest(returns=returns):
                self.g = GraphFixture(self.temp.name)
                g = self.g
                send = g.func('GCS::send_textv()', 8)
                service = g.func('GCS::service_statustext()', 100)
                g.lines.append('if (!vehicle_initialised()) {')
                g.edge(send, service, 'service_statustext();')
                for i, value in enumerate(returns):
                    predicate = g.func('GCS%u::vehicle_initialised()' % i)
                    if value is not None:
                        g.p.elf_returns[predicate.title] = value
                a = g.analyser()
                self.assertEqual(a.variant('rcin', 'rcin').worst(send)[0], expected)

    def test_statustext_guard_recognises_elf_only_predicate(self):
        g = self.g
        send = g.func('GCS::send_textv()', 8)
        service = g.func('GCS::service_statustext()', 100)
        predicate = '_ZNK3GCS19vehicle_initialisedEv'
        g.p.elf_returns[predicate] = 1
        g.p.demangled[predicate] = 'GCS::vehicle_initialised() const'
        g.lines.append('if (!vehicle_initialised()) {')
        g.edge(send, service, 'service_statustext();')
        self.assertEqual(g.analyser().variant('rcin', 'rcin').worst(send)[0], 8)

    def test_unguarded_statustext_call_and_unknown_source_remain_counted(self):
        for source in ('service_statustext();', None):
            with self.subTest(source=source):
                self.g = GraphFixture(self.temp.name)
                g = self.g
                send = g.func('GCS::send_textv()', 8)
                service = g.func('GCS::service_statustext()', 100)
                predicate = g.func('GCS::vehicle_initialised()')
                g.p.elf_returns[predicate.title] = 1
                g.lines.append('if (!vehicle_initialised()) {')
                g.edge(send, service, 'service_statustext();')
                g.lines.append('}')
                g.edge(send, service, source)
                a = g.analyser()
                self.assertEqual(a.variant('rcin', 'rcin').worst(send)[0], 108)

    def test_constant_returns_only_accept_unconditional_two_instruction_body(self):
        disassembly = '''00000010 <true>:
  10:\tmovs\tr0, #1
  12:\tbx\tlr
00000014 <conditional>:
  14:\tmoveq\tr0, #1
  16:\tbx\tlr
00000018 <more>:
  18:\tmovs\tr0, #1
  1a:\tbl\t10 <true>
  1e:\tbx\tlr
00000020 <last>:
  20:\tmovs\tr0, #0
  22:\tbx\tlr
'''
        with patch.object(sa.subprocess, 'run') as run:
            run.return_value.stdout = disassembly
            self.g.p.disassemble('unused')
        self.assertEqual(self.g.p.elf_returns, {'true': 1, 'last': 0})

    def test_constant_return_applies_to_all_symbol_aliases(self):
        disassembly = '''00000010 <first>:
  10:\tmovs\tr0, #1
  12:\tbx\tlr
'''
        self.g.p.elf_names_at[0x10] = ['first', 'second']
        with patch.object(sa.subprocess, 'run') as run:
            run.return_value.stdout = disassembly
            self.g.p.disassemble('unused')
        self.assertEqual(self.g.p.elf_returns, {'first': 1, 'second': 1})

    def test_linker_veneers_resolve_named_tail_calls(self):
        disassembly = '''00000010 <__flash_func_veneer>:
  10:\tpush\t{r0}
  12:\tldr\tr0, [pc, #8]
  14:\tmov\tip, r0
  16:\tpop\t{r0}
  18:\tbx\tip
00000020 <__other_func_veneer>:
  20:\tldr.w\tpc, [pc, #4]
00000028 <ordinary_indirect>:
  28:\tbx\tip
00000030 <__missing_func_veneer>:
  30:\tbx\tip
'''
        self.g.p.sym_values.update({'flash_func': (0x100, 8), 'other_func': (0x200, 8)})
        with patch.object(sa.subprocess, 'run') as run:
            run.return_value.stdout = disassembly
            self.g.p.disassemble('unused')
        self.assertEqual(self.g.p.elf_calls['__flash_func_veneer'], {'flash_func'})
        self.assertEqual(self.g.p.elf_calls['__other_func_veneer'], {'other_func'})
        self.assertEqual(self.g.p.elf_indirect['__flash_func_veneer'], 0)
        self.assertEqual(self.g.p.elf_indirect['__other_func_veneer'], 0)
        self.assertEqual(self.g.p.elf_indirect['ordinary_indirect'], 1)
        self.assertEqual(self.g.p.elf_indirect['__missing_func_veneer'], 1)

    def test_literal_thread_create_names_resolve_method_wrappers(self):
        g = self.g
        caller = g.func('Copter::one_hz_loop()')
        rate = g.func('void Functor<void>::method_wrapper<Copter, '
                      '&Copter::rate_controller_thread>(void*)')
        g.lines.extend([
            'hal.scheduler->thread_create(',
            '    FUNCTOR_BIND_MEMBER(&Copter::rate_controller_thread, void),',
            '    "rate", 1536, AP_HAL::Scheduler::PRIORITY_RCOUT, 1);',
        ])
        # GCC may attribute the call to the last line of a multi-line call.
        caller.edges.append((sa.INDIRECT, 'fixture.cpp:3:16'))
        entries = sa.registered_thread_entries(g.analyser())
        self.assertEqual(entries, {'rate': [rate]})

    def test_variable_thread_create_names_are_not_guessed(self):
        g = self.g
        caller = g.func('AP_DroneCAN::init()')
        g.func('void Functor<void>::method_wrapper<AP_DroneCAN, &AP_DroneCAN::loop>(void*)')
        g.lines.append('hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_DroneCAN::loop, void), '
                       '_thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0);')
        caller.edges.append((sa.INDIRECT, 'fixture.cpp:1:16'))
        self.assertEqual(sa.registered_thread_entries(g.analyser()), {})

    def test_suppression_parser_preserves_original_and_contextual_rules(self):
        path = os.path.join(self.temp.name, 'rules.txt')
        with open(path, 'w') as f:
            f.write('A -> B\n[!main] C -> D via E through F leaf # reason\n')
        self.assertEqual(sa.load_suppressions(path),
                         [(None, 'A', 'B', False, None, None),
                          ((True, {'main'}), 'C', 'D', True, 'E', 'F')])

    def test_lto_virtual_call_uses_inlined_source_function(self):
        g = self.g
        caller = g.func('outer()', 10)
        caller.tu = 'LTO'
        baro = g.func('AP_Baro_BMP::update()', 20)
        mode = g.func('ModeTakeoff::update()', 100)
        g.classes.bases = {'AP_Baro_Backend': [], 'AP_Baro_BMP': ['AP_Baro_Backend'],
                           'Mode': [], 'ModeTakeoff': ['Mode']}
        g.classes.slots = {'AP_Baro_Backend': {0: baro.title}, 'AP_Baro_BMP': {0: baro.title},
                           'Mode': {0: mode.title}, 'ModeTakeoff': {0: mode.title}}
        source = os.path.join(self.temp.name, 'libraries', 'AP_Baro')
        os.makedirs(source)
        with open(os.path.join(source, 'AP_Baro_Backend.cpp'), 'w') as f:
            f.write('update();\n')
        loc = 'libraries/AP_Baro/AP_Baro_Backend.cpp:1:7'
        inlined = 'AP_Baro_Backend::backend_update()'
        g.p.inline_sites[(caller.title, loc)] = {(inlined, caller.title)}
        g.p.poly_by_func[('libraries/AP_Baro/AP_Baro_Backend.cpp', inlined)] = {('AP_Baro_Backend', 0)}
        g.p.poly_by_tu['LTO'] = {('AP_Baro_Backend', 0), ('Mode', 0)}
        caller.edges.append((sa.INDIRECT, loc))
        self.assertEqual(g.analyser().variant('rcin', 'rcin').worst(caller)[0], 30)

    def test_addr2line_inline_chain_parser(self):
        output = ('0x100\nouter()\nouter.cpp:1\ninlined()\ninlined.h:2\n'
                  '0x104\n??\n??:0\nleaf()\nleaf.cpp:3\n')
        with patch.object(sa.subprocess, 'run') as run:
            run.return_value.stdout = output
            chains = sa.Program('arm-none-eabi-c++filt').addr2line_chains(
                'arm-none-eabi-addr2line', 'firmware', [0x100, 0x104])
        self.assertEqual(chains, {0x100: ['outer()', 'inlined()'], 0x104: ['leaf()']})
        self.assertEqual(run.call_args.kwargs['input'], '0x100\n0x104\n')

    def test_unlabelled_ci_edge_reaches_exact_partition(self):
        parent = '_Z3foov'
        part = '_Z3foov.part.0'
        ci = os.path.join(self.temp.name, 'fixture.cpp.0.ci')
        with open(ci, 'w') as f:
            f.write('node: { title: "%s" label: "foo()\\nfixture.cpp:2:1\\n16 bytes (static)" }\n' % parent)
            f.write('node: { title: "fixture.cpp:%s" '
                    'label: "foo()\\nfixture.cpp:2:1\\n200 bytes (static)" }\n' % part)
            f.write('edge: { sourcename: "%s" targetname: "fixture.cpp:%s" }\n' % (parent, part))
        p = sa.Program('unused')
        p.load_ci([ci])
        p.demangled = {parent: 'foo()', part: 'foo()'}
        p.demangle_all = lambda: None
        a = sa.Analyser(p, sa.ClassModel(), self.temp.name, self.temp.name, [], 'none')
        parent_func = p.funcs[parent]
        part_func = p.funcs['fixture.cpp:' + part]
        self.assertEqual(parent_func.edges, [('fixture.cpp:' + part, 'ci')])
        self.assertEqual(a.raw_succ[parent], [part_func])

    def test_elf_only_partition_is_not_resolved_to_its_parent(self):
        parent = '_Z3foov'
        part = '_Z3foov.part.0'
        p = sa.Program('unused')
        parent_func = p.add_ci_node(parent, 'foo()\\nfixture.cpp:2:1\\n16 bytes (static)', 'fixture')
        p.elf_frames[part] = 200
        p.add_elf_funcs()
        p.demangled = {parent: 'foo()', part: 'foo()'}
        a = sa.Analyser(p, sa.ClassModel(), self.temp.name, self.temp.name, [], 'none')
        targets, _ = a.targets(parent_func, part, 'ci')
        self.assertEqual([f.title for f in targets], [part])

    def test_lto_virtual_call_in_header_uses_inlined_function(self):
        g = self.g
        caller = g.func('outer()', 10)
        caller.tu = 'LTO'
        backend = g.func('Backend::queue_message()', 20)
        unrelated = g.func('Other::queue_message()', 100)
        g.classes.bases = {'Base': [], 'Backend': ['Base'], 'OtherBase': [], 'Other': ['OtherBase']}
        g.classes.slots = {'Base': {0: backend.title}, 'Backend': {0: backend.title},
                           'OtherBase': {0: unrelated.title}, 'Other': {0: unrelated.title}}
        source = os.path.join(self.temp.name, 'libraries', 'Frontend.h')
        os.makedirs(os.path.dirname(source))
        with open(source, 'w') as f:
            f.write('queue_message();\n')
        loc = 'libraries/Frontend.h:1:14'
        inlined = 'Frontend::queue_message()'
        g.p.inline_sites[(caller.title, loc)] = {(inlined, caller.title)}
        g.p.poly_by_symbol[inlined] = {('Base', 0)}
        caller.edges.append((sa.INDIRECT, loc))
        self.assertEqual(g.analyser().variant('rcin', 'rcin').worst(caller)[0], 30)

    def test_lto_suppression_matches_inlined_caller(self):
        g = self.g
        caller = g.func('outer()', 10)
        target = g.func('AP_MSP_Telem_Backend::process_packet()', 100)
        g.edge(caller, target, 'process_packet()')
        loc = 'fixture.cpp:1:15'
        g.p.inline_sites[(caller.title, loc)] = {('AP_RCTelemetry::run_wfq_scheduler()', caller.title)}
        rules = [(None, r'^AP_RCTelemetry::', r'^AP_MSP_Telem_Backend::', False, None, None)]
        self.assertEqual(g.analyser(rules).variant('rcin', 'rcin').worst(caller)[0], 10)

    def test_lto_inlined_shared_method_keeps_receiver(self):
        g = self.g
        frsky = g.func('AP_Frsky_SPort_Passthrough::get_telem_data()', 50)
        frsky.tu = 'LTO'
        cr_packet = g.func('AP_CRSF_Telem::process_packet()', 100)
        fr_packet = g.func('AP_Frsky_SPort_Passthrough::process_packet()', 40)
        g.classes.bases = {'AP_RCTelemetry': [], 'AP_CRSF_Telem': ['AP_RCTelemetry'],
                           'AP_Frsky_SPort_Passthrough': ['AP_RCTelemetry']}
        g.classes.slots = {'AP_RCTelemetry': {0: cr_packet.title},
                           'AP_CRSF_Telem': {0: cr_packet.title},
                           'AP_Frsky_SPort_Passthrough': {0: fr_packet.title}}
        loc = 'fixture.cpp:1:15'
        scheduler = 'AP_RCTelemetry::run_wfq_scheduler(bool)'
        g.p.inline_sites[(frsky.title, loc)] = {(scheduler, frsky.title)}
        g.p.poly_by_func[('fixture.cpp', scheduler)] = {('AP_RCTelemetry', 0)}
        g.lines.append('process_packet();')
        frsky.edges.append((sa.INDIRECT, loc))
        depth, path = g.analyser().variant('rcin', 'rcin').worst(frsky)
        self.assertEqual(depth, 90)
        self.assertEqual([f.name for f in path if isinstance(f, sa.Func)],
                         ['get_telem_data', 'process_packet'])

    def project_suppressions(self):
        return sa.load_suppressions(os.path.join(os.path.dirname(sa.__file__), 'stack_analysis_suppressions.txt'))

    def test_format_packets_keep_dynamic_emission_and_main_fmtu_recursion(self):
        g = self.g
        fmt = g.func('AP_Logger_Backend::Write_Format()', 10)
        fmtu = g.func('AP_Logger_Backend::Write_Format_Units()', 20)
        dynamic = g.func('AP_Logger_Backend::Write()', 30)
        block = g.func('AP_Logger_Backend::WriteCriticalBlock()', 10)
        prioritised = g.func('AP_Logger_Backend::WritePrioritisedBlock()', 10)
        ensure = g.func('AP_Logger_Backend::ensure_format_emitted()', 10)
        emit = g.func('AP_Logger_Backend::Write_Emit_FMT()', 100)
        for caller in (fmt, fmtu):
            g.edge(caller, block)
        g.edge(block, prioritised)
        g.edge(dynamic, prioritised)
        g.edge(prioritised, ensure)
        g.edge(ensure, emit)
        a = g.analyser(self.project_suppressions())
        for thread in ('main', 'rcin'):
            v = a.variant(thread, thread)
            self.assertEqual(v.worst(fmt)[0], 40)
            self.assertEqual(v.worst(dynamic)[0], 150)
            self.assertEqual(v.worst(fmtu)[0], 150 if thread == 'main' else 50)

    def test_assert_formatter_keeps_float_conversion_for_other_callers(self):
        g = self.g
        assertion = g.func('__assert_func', 8)
        ordinary = g.func('ordinary', 8)
        printf = g.func('fiprintf', 8)
        formatter = g.func('_vfiprintf_r', 8)
        floating = g.func('_printf_float', 100)
        for caller in (assertion, ordinary):
            g.edge(caller, printf)
        g.edge(printf, formatter)
        g.edge(formatter, floating)
        v = g.analyser(self.project_suppressions()).variant('main', 'main')
        self.assertEqual(v.worst(assertion)[0], 24)
        self.assertEqual(v.worst(ordinary)[0], 124)

    def test_slcan_delegation_keeps_hardware_and_canfd_keeps_override(self):
        g = self.g
        slcan = g.func('SLCAN::CANIface::send()', 8)
        hardware = g.func('ChibiOS::CANIface::send()', 80)
        g.edge(slcan, slcan)
        g.edge(slcan, hardware)
        init = g.func('ChibiOS::CANIface::init(unsigned long)', 8)
        fallback = g.func('AP_HAL::CANIface::init(unsigned long, unsigned long)', 8)
        override = g.func('ChibiOS::CANIface::init(unsigned long, unsigned long)', 80)
        g.edge(init, fallback)
        g.edge(init, override)
        g.edge(fallback, init)
        v = g.analyser(self.project_suppressions()).variant('main', 'main')
        self.assertEqual(v.worst(slcan)[0], 88)
        self.assertEqual(v.worst(init)[0], 88)
        self.assertFalse(v.unbounded)

    def test_idle_entry_matches_chibios_symbol(self):
        self.assertRegex('__idle_thread', re.compile(dict(sa.THREAD_ENTRIES)['idle']))

    def recursive_graph(self, call_limit=None):
        g = self.g
        recursive = g.func('recursive()', 24)
        exit_node = g.func('exit()', 80)
        g.edge(recursive, recursive)
        g.edge(recursive, exit_node)
        if call_limit is not None:
            g.annotation(recursive, call_limit)
        return g.analyser(), recursive

    def test_self_recursion_uses_simultaneous_call_limit(self):
        for call_limit in (1, 2, 5):
            with self.subTest(call_limit=call_limit):
                self.g = GraphFixture(self.temp.name)
                a, recursive = self.recursive_graph(call_limit)
                v = a.variant('rcin', 'rcin')
                self.assertEqual(v.worst(recursive)[0], call_limit * 24 + 80)
                self.assertFalse(v.unbounded)
                _, path = v.worst(recursive)
                self.assertEqual(sum(f.name == 'recursive' for f in path if isinstance(f, sa.Func)), call_limit)

    def test_unannotated_recursion_is_explicitly_unbounded(self):
        a, recursive = self.recursive_graph()
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.reachable([recursive])[2], v.unbounded)
        self.assertIn('UNBOUNDED', str(v.worst(recursive)[1]))

    def test_mutual_recursion_only_needs_a_bound_that_breaks_every_cycle(self):
        g = self.g
        first = g.func('first()', 10)
        second = g.func('second()', 20)
        exit_node = g.func('exit()', 100)
        g.edge(first, second)
        g.edge(second, first)
        g.edge(second, exit_node)
        g.annotation(first, 2, block=True)
        a = g.analyser()
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(first)[0], 160)
        self.assertEqual(v.worst(second)[0], 180)
        self.assertFalse(v.unbounded)

    def test_annotation_must_cover_an_unbounded_subcycle(self):
        g = self.g
        first = g.func('first()', 10)
        second = g.func('second()', 20)
        third = g.func('third()', 30)
        for caller, callee in ((first, second), (second, first), (second, third), (third, second)):
            g.edge(caller, callee)
        g.annotation(first, 2)
        v = g.analyser().variant('rcin', 'rcin')
        self.assertTrue(v.unbounded)

    def test_multiple_bounds_are_respected_together(self):
        g = self.g
        first = g.func('first()', 10)
        second = g.func('second()', 20)
        g.edge(first, second)
        g.edge(second, first)
        g.annotation(first, 3)
        g.annotation(second, 1)
        v = g.analyser().variant('rcin', 'rcin')
        self.assertEqual(v.worst(first)[0], 40)
        self.assertEqual(v.worst(second)[0], 30)

    def test_analysis_contexts_share_one_source_recursion_budget(self):
        g = self.g
        first = g.func('recursive()', 10)
        second = g.func('recursive()#context', 10)
        g.edge(first, second)
        g.edge(second, first)
        g.annotation(first, 3)
        second.loc = first.loc
        v = g.analyser().variant('rcin', 'rcin')
        self.assertEqual(v.worst(first)[0], 30)
        self.assertFalse(v.unbounded)

    def test_recursion_budget_is_shared_across_one_way_context_changes(self):
        g = self.g
        first = g.func('bounded()', 10)
        later = g.func('bounded()#later', 30)
        helper = g.func('helper()', 8)
        g.annotation(first, 2)
        later.loc = first.loc
        g.edge(first, first)
        g.edge(first, helper)
        g.edge(helper, later)
        g.edge(later, later)
        v = g.analyser().variant('main', 'main')
        # The two real SCCs cannot each spend a fresh two-call budget.
        self.assertEqual(v.worst(first)[0], 48)
        self.assertEqual(v.worst(later)[0], 60)
        self.assertFalse(v.unbounded)

    def test_partition_fragment_does_not_consume_its_callers_recursion_budget(self):
        g = self.g
        parent = g.func('recursive()', 16)
        part = g.func('recursive().part.0', 200)
        exit_node = g.func('exit()', 80)
        g.edge(parent, part)
        g.edge(part, parent)
        g.edge(part, exit_node)
        g.annotation(parent, 1)
        part.loc = parent.loc
        v = g.analyser().variant('rcin', 'rcin')
        self.assertEqual(v.worst(parent)[0], 296)
        self.assertFalse(v.unbounded)

    def test_large_bounded_cycles_use_a_safe_fallback(self):
        with patch.object(sa.Variant, 'bounded_paths', return_value=False):
            a, recursive = self.recursive_graph(3)
            v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(recursive)[0], 152)
        self.assertFalse(v.unbounded)

    def test_invalid_call_limit_is_rejected(self):
        for call_limit in ('0', '-1', '2.5', 'MAX_DEPTH', ''):
            with self.subTest(call_limit=call_limit):
                self.g = GraphFixture(self.temp.name)
                a, _ = self.recursive_graph(call_limit)
                with self.assertRaisesRegex(ValueError, 'positive integer'):
                    a.variant('rcin', 'rcin')

    def test_fallback_accounts_for_uncapped_helpers_between_bounded_calls(self):
        g = self.g
        first = g.func('first()', 10)
        second = g.func('second()', 20)
        exit_node = g.func('exit()', 100)
        g.edge(first, second)
        g.edge(second, first)
        g.edge(second, exit_node)
        g.annotation(first, 2)
        with patch.object(sa.Variant, 'bounded_paths', return_value=False):
            v = g.analyser().variant('rcin', 'rcin')
        self.assertEqual(v.worst(second)[0], 180)
        self.assertFalse(v.unbounded)

    def test_duplicate_annotations_are_rejected(self):
        for comment in ('// @StackCallLimit: 2 @StackCallLimit: 3',
                        '// @StackCallLimit: 2\n// @StackCallLimit: 3'):
            with self.subTest(comment=comment):
                self.g = GraphFixture(self.temp.name)
                g = self.g
                recursive = g.func('recursive()')
                g.edge(recursive, recursive)
                g.lines.extend(comment.splitlines() + ['void recursive() {}'])
                recursive.loc = 'fixture.cpp:%u:1' % len(g.lines)
                with self.assertRaisesRegex(ValueError, 'must occur once'):
                    g.analyser().variant('rcin', 'rcin')

    def test_annotation_above_a_multiline_template_signature(self):
        g = self.g
        recursive = g.func('recursive()')
        g.edge(recursive, recursive)
        g.lines.extend(['// @StackCallLimit: 2', '// At most one nested call.',
                        'template<typename T>', 'void', 'recursive()', '{}'])
        recursive.loc = 'fixture.cpp:5:1'
        v = g.analyser().variant('rcin', 'rcin')
        self.assertEqual(v.worst(recursive)[0], 16)
        self.assertFalse(v.unbounded)

    def test_annotation_does_not_leak_from_previous_function(self):
        g = self.g
        previous = g.func('previous()')
        recursive = g.func('recursive()')
        g.edge(recursive, recursive)
        g.annotation(previous, 2)
        g.lines.append('void recursive() {}')
        recursive.loc = 'fixture.cpp:%u:1' % len(g.lines)
        self.assertTrue(g.analyser().variant('rcin', 'rcin').unbounded)

    def test_strict_check_fails_without_bounds_and_counts_annotated_depth(self):
        for call_limit, allocation, expected in ((None, 1000, 1), (3, 1000, 0), (3, 80, 1)):
            with self.subTest(call_limit=call_limit, allocation=allocation):
                self.g = GraphFixture(self.temp.name)
                g = self.g
                root = g.func('ChibiOS::Scheduler::_rcin_thread(void*)', 16)
                recursive = g.func('recursive()', 24)
                g.edge(root, recursive)
                g.edge(recursive, recursive)
                if call_limit is not None:
                    g.annotation(recursive, call_limit)
                g.analyser()  # write the fixture source
                output = StringIO()
                with ExitStack() as stack:
                    stack.enter_context(patch.object(sa, 'Program', return_value=g.p))
                    stack.enter_context(patch.object(sa, 'ClassModel', return_value=g.classes))
                    stack.enter_context(patch.object(sa.glob, 'glob', return_value=['fixture.ci']))
                    for name in ('load_ci', 'load_cgraph_dumps', 'load_elf', 'add_elf_funcs', 'demangle_all'):
                        stack.enter_context(patch.object(g.p, name))
                    stack.enter_context(patch.object(g.p, 'remove_unlinked', return_value=0))
                    stack.enter_context(patch.object(g.classes, 'load'))
                    stack.enter_context(patch.object(sa, 'static_allocations', return_value={'rcin': allocation}))
                    argv = ['stack_analysis', g.root, '--srcroot', g.root, '--elf', 'unused', '--check']
                    stack.enter_context(patch.object(sa.sys, 'argv', argv))
                    stack.enter_context(redirect_stdout(output))
                    if expected:
                        with self.assertRaises(SystemExit) as error:
                            sa.main()
                        self.assertEqual(error.exception.code, expected)
                    else:
                        sa.main()
                if call_limit is None:
                    self.assertIn('UNBOUNDED RECURSION:', output.getvalue())
                elif allocation == 80:
                    # 16 bytes of caller + three 24-byte recursive frames.
                    self.assertIn('STACK OVERFLOW: rcin needs up to 88 bytes', output.getvalue())

    def test_strict_check_ignores_unsized_dynamic_thread_recursion(self):
        g = self.g
        g.func('ChibiOS::Scheduler::_rcin_thread(void*)', 16)
        creator = g.func('Copter::one_hz_loop()', 8)
        scripting = g.func('void Functor<void>::method_wrapper<AP_Scripting, '
                           '&AP_Scripting::thread>(void*)', 24)
        recursive = g.func('recursive()', 32)
        g.edge(scripting, recursive)
        g.edge(recursive, recursive)
        g.lines.append('hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_Scripting::thread, void), '
                       '"Scripting", 2048, AP_HAL::Scheduler::PRIORITY_SCRIPTING, 0);')
        creator.edges.append((sa.INDIRECT, 'fixture.cpp:1:16'))
        g.analyser()  # write the fixture source
        output = StringIO()
        with ExitStack() as stack:
            stack.enter_context(patch.object(sa, 'Program', return_value=g.p))
            stack.enter_context(patch.object(sa, 'ClassModel', return_value=g.classes))
            stack.enter_context(patch.object(sa.glob, 'glob', return_value=['fixture.ci']))
            for name in ('load_ci', 'load_cgraph_dumps', 'load_elf', 'add_elf_funcs', 'demangle_all'):
                stack.enter_context(patch.object(g.p, name))
            stack.enter_context(patch.object(g.p, 'remove_unlinked', return_value=0))
            stack.enter_context(patch.object(g.classes, 'load'))
            stack.enter_context(patch.object(sa, 'static_allocations', return_value={'rcin': 1000}))
            argv = ['stack_analysis', g.root, '--srcroot', g.root, '--elf', 'unused', '--check']
            stack.enter_context(patch.object(sa.sys, 'argv', argv))
            stack.enter_context(redirect_stdout(output))
            sa.main()
        self.assertIn('Scripting', output.getvalue())
        self.assertNotIn('UNBOUNDED RECURSION:', output.getvalue())

    def test_strict_check_rejects_unsized_explicit_thread_recursion(self):
        g = self.g
        g.func('ChibiOS::Scheduler::_rcin_thread(void*)', 16)
        uart = g.func('ChibiOS::UARTDriver::uart_thread()', 24)
        recursive = g.func('recursive()', 32)
        g.edge(uart, recursive)
        g.edge(recursive, recursive)
        g.analyser()  # write the fixture source
        output = StringIO()
        with ExitStack() as stack:
            stack.enter_context(patch.object(sa, 'Program', return_value=g.p))
            stack.enter_context(patch.object(sa, 'ClassModel', return_value=g.classes))
            stack.enter_context(patch.object(sa.glob, 'glob', return_value=['fixture.ci']))
            for name in ('load_ci', 'load_cgraph_dumps', 'load_elf', 'add_elf_funcs', 'demangle_all'):
                stack.enter_context(patch.object(g.p, name))
            stack.enter_context(patch.object(g.p, 'remove_unlinked', return_value=0))
            stack.enter_context(patch.object(g.classes, 'load'))
            stack.enter_context(patch.object(sa, 'static_allocations', return_value={'rcin': 1000}))
            argv = ['stack_analysis', g.root, '--srcroot', g.root, '--elf', 'unused', '--check']
            stack.enter_context(patch.object(sa.sys, 'argv', argv))
            stack.enter_context(redirect_stdout(output))
            with self.assertRaises(SystemExit) as error:
                sa.main()
            self.assertEqual(error.exception.code, 1)
        self.assertIn('UNBOUNDED RECURSION: UART', output.getvalue())

    def test_external_recursion_annotations_are_specific_to_elf_functions(self):
        g = self.g
        first = g.func('first()', 10)
        second = g.func('second()', 20)
        first.origin = second.origin = 'elf'
        first.loc = second.loc = 'elf'
        g.edge(first, first)
        g.edge(second, second)
        a = g.analyser(call_limits=[(re.compile('^first'), 3), (re.compile('^second'), 2)])
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(first)[0], 30)
        self.assertEqual(v.worst(second)[0], 40)
        self.assertFalse(v.unbounded)

    def test_external_annotation_does_not_replace_a_missing_source_comment(self):
        g = self.g
        recursive = g.func('recursive()', 10)
        g.edge(recursive, recursive)
        a = g.analyser(call_limits=[(re.compile('^recursive'), 3)])
        self.assertTrue(a.variant('rcin', 'rcin').unbounded)

    def delay_callback_graph(self):
        g = self.g
        root = g.func('root()', 8)
        delay = g.func('ChibiOS::Scheduler::delay(unsigned short)', 10)
        dispatcher = g.func('AP_HAL::Scheduler::call_delay_cb()', 20)
        callback = g.func('AP_Vehicle::scheduler_delay_callback()', 30)
        worker = g.func('worker()', 100)
        g.annotation(callback, 1)
        g.edge(root, delay)
        g.edge(delay, dispatcher)
        g.edge(dispatcher, sa.INDIRECT, '_delay_cb()')
        g.edge(callback, worker)
        g.edge(worker, delay)
        return root, delay, dispatcher, callback

    def test_delay_callback_is_counted_once_in_main(self):
        root, delay, dispatcher, callback = self.delay_callback_graph()
        a = self.g.analyser(self.project_suppressions())
        self.assertEqual(a.raw_succ[dispatcher.title], [callback])
        depth, path = a.variant('main', 'main').worst(root)
        # A nested delay still creates its own dispatcher frame, but the
        # callback's non-reentrant guard prevents a second callback body.
        self.assertEqual(depth, 198)
        self.assertEqual(sum(isinstance(f, sa.Func) and f.name == 'scheduler_delay_callback' for f in path), 1)
        self.assertFalse(a.variant('main', 'main').unbounded)

    def test_delay_callback_does_not_run_in_other_threads(self):
        root, delay, dispatcher, callback = self.delay_callback_graph()
        a = self.g.analyser(self.project_suppressions())
        depth, path = a.variant('rcin', 'rcin').worst(root)
        self.assertEqual(depth, 38)
        self.assertNotIn(callback, path)

    def test_fatal_recovery_does_not_reenable_delay_callback(self):
        root, delay, dispatcher, callback = self.delay_callback_graph()
        fatal = self.g.func('AP_BoardConfig::throw_error_loop(char const*, char const*, va_list)', 16)
        self.g.edge(fatal, delay)
        a = self.g.analyser(self.project_suppressions())
        self.assertEqual(a.variant('main', 'main').worst(fatal)[0], 46)
        # Ordinary callers retain the callback and its worker.
        self.assertEqual(a.variant('main', 'main').worst(root)[0], 198)

    def test_storage_startup_is_kept_but_not_repeated_from_main_loop(self):
        g = self.g
        startup = g.func('AP_Vehicle::load_parameters(version, expected)', 10)
        loop = g.func('AP_Vehicle::loop()', 20)
        vehicle_init = g.func('Plane::init_ardupilot()', 20)
        callback = g.func('AP_Vehicle::scheduler_delay_callback()', 30)
        read = g.func('ChibiOS::Storage::read_block(dst, loc, n)', 8)
        storage = g.func('ChibiOS::Storage::_storage_open()', 16)
        backup = g.func('ChibiOS::Storage::_save_backup()', 100)
        dispatcher = g.func('AP_HAL::Scheduler::call_delay_cb()', 4)
        g.edge(startup, read)
        g.edge(loop, read)
        g.edge(vehicle_init, read)
        g.edge(callback, read)
        g.edge(read, storage)
        g.edge(storage, backup)
        g.edge(backup, dispatcher)
        g.edge(dispatcher, sa.INDIRECT, '_delay_cb()')
        v = g.analyser(self.project_suppressions()).variant('main', 'main')
        self.assertEqual(v.worst(startup)[0], 138)
        self.assertEqual(v.worst(loop)[0], 44)
        self.assertEqual(v.worst(vehicle_init)[0], 44)
        self.assertEqual(v.worst(callback)[0], 54)
        self.assertEqual(v.worst(dispatcher)[0], 58)

    def test_hal_defaults_reload_uses_romfs_without_hiding_other_file_loads(self):
        g = self.g
        reload = g.func('AP_Param::reload_defaults_file(bool)', 10)
        load = g.func('AP_Param::load_defaults_file(filename, bool)', 20)
        fs = g.func('AP_Filesystem::open(filename, flags, bool)', 8)
        romfs = g.func('AP_Filesystem_ROMFS::open(filename, flags, bool)', 16)
        param = g.func('AP_Filesystem_Param::open(filename, flags, bool)', 100)
        fatfs = g.func('AP_Filesystem_FATFS::open(filename, flags, bool)', 200)
        g.edge(reload, load)
        g.edge(load, fs)
        for backend in (romfs, param, fatfs):
            g.edge(fs, backend)
        v = g.analyser(self.project_suppressions()).variant('main', 'main')
        self.assertEqual(v.worst(reload)[0], 54)
        self.assertEqual(v.worst(load)[0], 228)

    def test_plane_aux_init_skips_mode_changes_without_hiding_other_aux_calls(self):
        g = self.g
        init = g.func('RC_Channel_Plane::init_aux_function(option, pos)', 10)
        run = g.func('RC_Channel::run_aux_function(option, pos)', 20)
        dispatch = g.func('RC_Channel_Plane::do_aux_function(trigger)', 30)
        change = g.func('RC_Channel_Plane::do_aux_function_change_mode(mode, pos)', 40)
        reset = g.func('RC_Channels::reset_mode_switch()', 100)
        mode = g.func('Plane::set_mode_by_number(mode, reason)', 200)
        other = g.func('Other::aux_function()', 50)
        g.edge(init, run)
        g.edge(run, dispatch)
        g.edge(dispatch, change)
        g.edge(change, reset)
        g.edge(dispatch, reset)  # MODE_SWITCH_RESET, or an inlined helper.
        g.edge(dispatch, mode)
        g.edge(dispatch, other)
        v = g.analyser(self.project_suppressions()).variant('main', 'main')
        self.assertEqual(v.worst(init)[0], 110)
        self.assertEqual(v.worst(dispatch)[0], 230)

    def test_autoland_command_dispatch_cannot_start_a_vtol_takeoff(self):
        g = self.g
        autoland = g.func('ModeAutoLand::_enter()', 10)
        update = g.func('ModeAutoLand::update()', 12)
        start = g.func('Plane::start_command(cmd)', 20)
        takeoff = g.func('QuadPlane::do_vtol_takeoff(cmd)', 100)
        land = g.func('Plane::do_land(cmd)', 30)
        vtol_land = g.func('QuadPlane::do_vtol_land(cmd)', 100)
        g.edge(autoland, start)
        g.edge(update, start)
        g.edge(start, takeoff)
        g.edge(start, vtol_land)
        g.edge(start, land)
        v = g.analyser(self.project_suppressions()).variant('main', 'main')
        self.assertEqual(v.worst(autoland)[0], 60)
        self.assertEqual(v.worst(update)[0], 132)
        self.assertEqual(v.worst(start)[0], 120)

    def test_config_error_can_disarm_but_cannot_enter_arming_checks(self):
        g = self.g
        restricted = g.func('GCS_MAVLINK::handle_config_error_message(status, msg)', 10)
        handler = g.func('GCS_MAVLINK::handle_command_component_arm_disarm(packet)', 20)
        arm = g.func('AP_Arming_Plane::arm(method, bool)', 100)
        disarm = g.func('AP_Arming_Plane::disarm(method, bool)', 30)
        g.edge(restricted, handler)
        g.edge(handler, arm)
        g.edge(handler, disarm)
        v = g.analyser(self.project_suppressions()).variant('main', 'main')
        self.assertEqual(v.worst(restricted)[0], 60)
        self.assertEqual(v.worst(handler)[0], 120)

    def test_inlined_delay_dispatcher_resolves_its_callback(self):
        g = self.g
        outer = g.func('outer()', 10)
        callback = g.func('AP_Vehicle::scheduler_delay_callback()', 30)
        g.edge(outer, sa.INDIRECT, '_delay_cb()')
        loc = outer.edges[0][1]
        g.p.inline_sites[(outer.title, loc)] = {('AP_HAL::Scheduler::call_delay_cb()', outer.title)}
        a = g.analyser()
        self.assertEqual(a.raw_succ[outer.title], [callback])

    def test_mangled_lto_delay_dispatcher_resolves_its_callback(self):
        g = self.g
        outer = g.func('outer()', 10)
        callback = g.func('AP_Vehicle::scheduler_delay_callback()', 30)
        g.edge(outer, sa.INDIRECT, '_delay_cb()')
        loc = outer.edges[0][1]
        dispatcher = '_ZN6AP_HAL9Scheduler13call_delay_cbEv'
        g.p.inline_sites[(outer.title, loc)] = {(dispatcher, outer.title)}
        g.p.demangled[dispatcher] = 'AP_HAL::Scheduler::call_delay_cb()'
        self.assertEqual(g.analyser().raw_succ[outer.title], [callback])

    def test_unrelated_delay_pointer_stays_unresolved(self):
        g = self.g
        other = g.func('Other::call()', 10)
        g.func('AP_Vehicle::scheduler_delay_callback()', 30)
        g.edge(other, sa.INDIRECT, '_delay_cb()')
        a = g.analyser()
        self.assertEqual(a.raw_succ[other.title], [])
        self.assertTrue(a.unresolved[other.title])

    def test_mixed_vehicle_graphs_are_rejected_even_when_older_than_elf(self):
        first = os.path.join(self.temp.name, 'Shared.cpp.0.ci')
        second = os.path.join(self.temp.name, 'Shared.cpp.1.ci')
        elf = os.path.join(self.temp.name, 'firmware')
        for path in (first, second, elf):
            with open(path, 'w'):
                pass
        os.utime(first, ns=(1, 1))
        os.utime(second, ns=(1, 1))
        os.utime(elf, ns=(2, 2))
        self.assertEqual(sa.graph_files_newer_than_elf([first, second], elf), [])
        for files in ([first, second], [second, first]):
            with self.assertRaisesRegex(ValueError, 'multiple call graphs.*clean the board'):
                sa.validate_graph_files(files)

    def test_graph_validation_keeps_distinct_sources_and_lto_partitions(self):
        sa.validate_graph_files(['a.cpp.0.ci', 'b.cpp.1.ci'])
        sa.validate_graph_files(['firmware.ltrans0.ltrans.ci', 'firmware.ltrans1.ltrans.ci'])
        with self.assertRaisesRegex(ValueError, 'multiple call graphs'):
            sa.validate_graph_files(['a.cpp.0.000i.cgraph', 'a.cpp.1.000i.cgraph'])

    def test_real_lto_elf_recovers_inline_source_calls(self):
        tools = ('gcc', 'c++filt', 'objdump', 'addr2line')
        if not all(shutil.which('arm-none-eabi-' + tool) for tool in tools):
            self.skipTest('ARM toolchain required for the ELF/DWARF integration test')
        if importlib.util.find_spec('elftools') is None:
            self.skipTest('pyelftools required for the ELF/DWARF integration test')
        source = os.path.join(self.temp.name, 'inline.c')
        elf = os.path.join(self.temp.name, 'inline.elf')
        with open(source, 'w') as f:
            f.write('__attribute__((noinline)) int leaf(int x) { asm volatile("" ::: "memory"); return x + 1; }\n'
                    'static inline __attribute__((always_inline)) int helper(int x) { return leaf(x) + 2; }\n'
                    '__attribute__((used)) int entry(int x) { return helper(x) + 3; }\n')
        subprocess.run(['arm-none-eabi-gcc', '-mcpu=cortex-m4', '-mthumb', '-g', '-O2', '-flto',
                        '-nostdlib', '-Wl,-e,entry', source, '-o', elf], check=True, capture_output=True, text=True)
        p = sa.Program('arm-none-eabi-c++filt')
        p.lto = True
        p.load_elf(elf)
        self.assertTrue(any(callee == 'leaf' for _, _, callee in p.elf_call_sites))
        sites = [(loc, chain) for (caller, loc), chains in p.inline_sites.items()
                 if caller == 'entry' for chain in chains if 'helper' in chain and 'entry' in chain]
        self.assertTrue(sites, p.inline_sites)
        self.assertTrue(all(sa.source_tu(loc) == source for loc, _ in sites))

    def test_graph_files_newer_than_elf(self):
        elf = os.path.join(self.temp.name, 'firmware')
        old_graph = os.path.join(self.temp.name, 'old.ci')
        new_graph = os.path.join(self.temp.name, 'new.ci')
        for path in (old_graph, elf, new_graph):
            with open(path, 'w'):
                pass
        os.utime(old_graph, ns=(1, 1))
        os.utime(elf, ns=(2, 2))
        os.utime(new_graph, ns=(3, 3))
        self.assertEqual(sa.graph_files_newer_than_elf([old_graph, new_graph], elf), [new_graph])

    def test_external_annotation_parser_rejects_bad_values_and_regexes(self):
        path = os.path.join(self.temp.name, 'bounds.txt')
        for annotation in ('# @StackCallLimit: 0 ^f$', '# @StackCallLimit: 2', '# @StackCallLimit: 2 ['):
            with self.subTest(annotation=annotation):
                with open(path, 'w') as f:
                    f.write(annotation + '\n')
                with self.assertRaises(ValueError):
                    sa.load_call_limits(path)
        with open(path, 'w') as f:
            f.write('# Explanation\n# @StackCallLimit: 2 ^f$\n')
        pattern, bound = sa.load_call_limits(path)[0]
        self.assertEqual(bound, 2)
        self.assertRegex('f', pattern)


if __name__ == '__main__':
    unittest.main()
