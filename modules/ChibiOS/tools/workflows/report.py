#!/usr/bin/env python

"""Scan test results directory and report status."""

import argparse
import os
import sys

from xml.etree import ElementTree


def report(args, fd):

    def write_brief_status(fd, testcase, status):
        fd.write('{makefile} ({target}) ... {status} {message}\n'.format(
            makefile=testcase.attrib['file'],
            target=testcase.attrib['name'],
            status=status.attrib['type'],
            message=status.attrib['message']
        ))

    def write_brief_ok(fd, testcase):
        fd.write('{makefile} ({target}) ... ok\n'.format(
            makefile=testcase.attrib['file'],
            target=testcase.attrib['name'],
        ))

    def write_failed_status(fd, testcase):
        fd.write(thick_line)
        fd.write('FAIL: {makefile} ({target})\n'.format(
            makefile=testcase.attrib['file'],
            target=testcase.attrib['name'],
        ))
        fd.write(thin_line)
        fd.write('CMD: {}\n'.format(testcase.attrib['log']))
        stdout = testcase.find('system-out')
        if stdout is not None:
            fd.write('\nSTDOUT:\n')
            fd.write(stdout.text)
        stderr = testcase.find('system-err')
        if stderr is not None:
            fd.write('\nSTDERR:\n')
            fd.write(stderr.text)
        fd.write('\n')

    def write(fds, string):
        for fd in fds:
            fd.write(string)

    thick_line = '=' * 70 + '\n'
    thin_line = '-' * 70 + '\n'

    time = 0
    total_count = 0
    skipped_count = 0
    failures = []

    for root_dir, _, files in os.walk(args.test_results):
        for file in files:
            if file.endswith('.xml'):
                path = os.path.join(root_dir, file)
                tree = ElementTree.parse(path)
                for testcase in tree.iter('testcase'):
                    time += float(testcase.attrib['time'])
                    total_count += 1

                    fail = testcase.find('failure')
                    if fail is not None:
                        if args.verbose == 1:
                            write_brief_status(fd, testcase, fail)

                        failures.append(testcase)
                        continue

                    skip = testcase.find('skipped')
                    if skip is not None:
                        if args.verbose == 1:
                            write_brief_status(fd, testcase, skip)
                        skipped_count += 1
                        continue

                    if args.verbose == 1:
                        write_brief_ok(fd, testcase)

    failures_count = len(failures)

    if args.verbose == 1 and (failures_count > 0 or skipped_count > 0):
        fd.write('\n')

    for testcase in failures:
        write_failed_status(fd, testcase)

    fds = [fd]
    if fd != sys.stdout:
        fds.append(sys.stdout)

    write(fds, thin_line)
    write(fds, 'Ran {total:d} tests in {time:.3f}s\n\n'.format(
        total=total_count,
        time=time,
    ))

    context = ', '.join(
        '{}={}'.format(*v)
        for v in [
            ('failures', failures_count),
            ('skipped', skipped_count),
        ]
        if v[1] > 0
    )

    if failures_count > 0:
        write(fds, 'FAILED ({})\n'.format(context))
        sys.exit(1)

    if context:
        write(fds, 'OK ({})\n'.format(context))
        sys.exit(0)

    write(fds, 'OK\n')


def main():
    parser = argparse.ArgumentParser(description=(
        'Scan test results and report status'
    ))
    parser.add_argument('-v', '--verbose', action='count', default=0)
    parser.add_argument('-t', '--test-results',
                        help='Directory with JUnit XML test results')
    parser.add_argument('-r', '--report', default='-',
                        help='Filename to store test report')
    args = parser.parse_args()

    if args.report == '-':
        report(args, sys.stdout)
    else:
        with open(args.report, 'w') as fd:
            report(args, fd)


if __name__ == '__main__':
    main()
