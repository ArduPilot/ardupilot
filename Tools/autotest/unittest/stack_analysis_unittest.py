#!/usr/bin/env python3
"""Regression tests for conservative stack call graph refinements.

AP_FLAKE8_CLEAN
"""

import os
import re
import tempfile
import unittest
from contextlib import ExitStack, redirect_stdout
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

    def analyser(self, rules=(), recursion_limits=()):
        with open(os.path.join(self.root, 'fixture.cpp'), 'w') as f:
            f.write('\n'.join(self.lines) + '\n')
        self.classes.index()
        return sa.Analyser(self.p, self.classes, self.root, self.root, [], 'none', rules, recursion_limits)

    def annotation(self, f, value, block=False):
        if block:
            self.lines.extend(['/*', ' * @StackMaxRecursion: %s' % value, ' * Bound justified by the caller.', ' */'])
        else:
            self.lines.append('// @StackMaxRecursion: %s' % value)
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

    def recursive_graph(self, bound=None):
        g = self.g
        recursive = g.func('recursive()', 24)
        exit_node = g.func('exit()', 80)
        g.edge(recursive, recursive)
        g.edge(recursive, exit_node)
        if bound is not None:
            g.annotation(recursive, bound)
        return g.analyser(), recursive

    def test_self_recursion_uses_maximum_simultaneous_invocations(self):
        for bound in (1, 2, 5):
            with self.subTest(bound=bound):
                self.g = GraphFixture(self.temp.name)
                a, recursive = self.recursive_graph(bound)
                v = a.variant('rcin', 'rcin')
                self.assertEqual(v.worst(recursive)[0], bound * 24 + 80)
                self.assertFalse(v.unbounded)
                _, path = v.worst(recursive)
                self.assertEqual(sum(f.name == 'recursive' for f in path if isinstance(f, sa.Func)), bound)

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

    def test_large_bounded_cycles_use_a_safe_fallback(self):
        with patch.object(sa.Variant, 'bounded_paths', return_value=False):
            a, recursive = self.recursive_graph(3)
            v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(recursive)[0], 152)
        self.assertFalse(v.unbounded)

    def test_invalid_recursion_bound_is_rejected(self):
        for bound in ('0', '-1', '2.5', 'MAX_DEPTH', ''):
            with self.subTest(bound=bound):
                self.g = GraphFixture(self.temp.name)
                a, _ = self.recursive_graph(bound)
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
        for comment in ('// @StackMaxRecursion: 2 @StackMaxRecursion: 3',
                        '// @StackMaxRecursion: 2\n// @StackMaxRecursion: 3'):
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
        g.lines.extend(['// @StackMaxRecursion: 2', '// At most one nested call.',
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
        for bound, allocation, expected in ((None, 1000, 1), (3, 1000, 0), (3, 80, 1)):
            with self.subTest(bound=bound, allocation=allocation):
                self.g = GraphFixture(self.temp.name)
                g = self.g
                root = g.func('ChibiOS::Scheduler::_rcin_thread(void*)', 16)
                recursive = g.func('recursive()', 24)
                g.edge(root, recursive)
                g.edge(recursive, recursive)
                if bound is not None:
                    g.annotation(recursive, bound)
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
                if bound is None:
                    self.assertIn('UNBOUNDED RECURSION:', output.getvalue())
                elif allocation == 80:
                    # 16 bytes of caller + three 24-byte recursive frames.
                    self.assertIn('STACK OVERFLOW: rcin needs up to 88 bytes', output.getvalue())

    def test_external_recursion_annotations_are_specific_to_elf_functions(self):
        g = self.g
        first = g.func('first()', 10)
        second = g.func('second()', 20)
        first.origin = second.origin = 'elf'
        first.loc = second.loc = 'elf'
        g.edge(first, first)
        g.edge(second, second)
        a = g.analyser(recursion_limits=[(re.compile('^first'), 3), (re.compile('^second'), 2)])
        v = a.variant('rcin', 'rcin')
        self.assertEqual(v.worst(first)[0], 30)
        self.assertEqual(v.worst(second)[0], 40)
        self.assertFalse(v.unbounded)

    def test_external_annotation_does_not_replace_a_missing_source_comment(self):
        g = self.g
        recursive = g.func('recursive()', 10)
        g.edge(recursive, recursive)
        a = g.analyser(recursion_limits=[(re.compile('^recursive'), 3)])
        self.assertTrue(a.variant('rcin', 'rcin').unbounded)

    def test_external_annotation_parser_rejects_bad_values_and_regexes(self):
        path = os.path.join(self.temp.name, 'bounds.txt')
        for annotation in ('# @StackMaxRecursion: 0 ^f$', '# @StackMaxRecursion: 2', '# @StackMaxRecursion: 2 ['):
            with self.subTest(annotation=annotation):
                with open(path, 'w') as f:
                    f.write(annotation + '\n')
                with self.assertRaises(ValueError):
                    sa.load_recursion_limits(path)
        with open(path, 'w') as f:
            f.write('# Explanation\n# @StackMaxRecursion: 2 ^f$\n')
        pattern, bound = sa.load_recursion_limits(path)[0]
        self.assertEqual(bound, 2)
        self.assertRegex('f', pattern)


if __name__ == '__main__':
    unittest.main()
