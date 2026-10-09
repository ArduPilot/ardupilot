#!/usr/bin/env python3
"""Regression tests for conservative stack call graph refinements.

AP_FLAKE8_CLEAN
"""

import os
import re
import tempfile
import unittest
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

    def analyser(self, rules=()):
        with open(os.path.join(self.root, 'fixture.cpp'), 'w') as f:
            f.write('\n'.join(self.lines) + '\n')
        self.classes.index()
        return sa.Analyser(self.p, self.classes, self.root, self.root, [], 'none', rules)


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

    def test_suppression_parser_preserves_original_and_contextual_rules(self):
        path = os.path.join(self.temp.name, 'rules.txt')
        with open(path, 'w') as f:
            f.write('A -> B\n[!main] C -> D via E through F leaf # reason\n')
        self.assertEqual(sa.load_suppressions(path),
                         [(None, 'A', 'B', False, None, None),
                          ((True, {'main'}), 'C', 'D', True, 'E', 'F')])

    def test_idle_entry_matches_chibios_symbol(self):
        self.assertRegex('__idle_thread', re.compile(dict(sa.THREAD_ENTRIES)['idle']))


if __name__ == '__main__':
    unittest.main()
