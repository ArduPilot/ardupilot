#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Opt-in test of VS Code's real C/C++ debug adapter against USB firmware.

Set USB_DEBUG_WORKSPACE to a generated .code-workspace and USB_DEBUG_ADAPTER to
OpenDebugAD7 (OpenDebugAD7.exe on Windows). The matching firmware must already
be loaded and connected. This test resumes execution and sets a Plane breakpoint.
"""

import json
import os
import queue
import subprocess
import threading
import time
import unittest

from pathlib import Path


class Adapter:
    def __init__(self, executable):
        self.process = subprocess.Popen([executable], stdin=subprocess.PIPE,
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        self.messages = queue.Queue()
        self.pending = []
        self.sequence = 0
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        while True:
            length = None
            while True:
                line = self.process.stdout.readline()
                if not line:
                    self.messages.put({'event': 'adapterExited'})
                    return
                if line.lower().startswith(b'content-length:'):
                    length = int(line.split(b':')[1])
                if line == b'\r\n':
                    break
            self.messages.put(json.loads(self.process.stdout.read(length)))

    def send(self, command, arguments=None):
        self.sequence += 1
        data = json.dumps({'seq': self.sequence, 'type': 'request', 'command': command,
                           'arguments': arguments or {}}).encode()
        self.process.stdin.write(f'Content-Length: {len(data)}\r\n\r\n'.encode()+data)
        self.process.stdin.flush()
        return self.sequence

    def wait(self, predicate, timeout=60):
        deadline = time.monotonic()+timeout
        while True:
            for index, message in enumerate(self.pending):
                if predicate(message):
                    return self.pending.pop(index)
            message = self.messages.get(timeout=max(.01, deadline-time.monotonic()))
            print(json.dumps(message), flush=True)
            if message.get('event') == 'adapterExited':
                raise RuntimeError('debug adapter exited')
            if message.get('type') == 'response' and not message.get('success'):
                raise RuntimeError(message)
            self.pending.append(message)

    def response(self, sequence):
        return self.wait(lambda m: m.get('request_seq') == sequence)

    def request(self, command, arguments=None):
        return self.response(self.send(command, arguments)).get('body', {})

    def event(self, name):
        return self.wait(lambda m: m.get('event') == name).get('body', {})

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process.stdin.close()
        self.process.stdout.close()


@unittest.skipUnless(os.environ.get('USB_DEBUG_WORKSPACE') and os.environ.get('USB_DEBUG_ADAPTER'),
                     'set USB_DEBUG_WORKSPACE and USB_DEBUG_ADAPTER for the hardware/ Renode adapter test')
class VSCodeTests(unittest.TestCase):
    def test_attach_debug_and_detach(self):
        config = json.loads(Path(os.environ['USB_DEBUG_WORKSPACE']).read_text())['launch']['configurations'][0]
        adapter = Adapter(os.environ['USB_DEBUG_ADAPTER'])
        try:
            adapter.request('initialize', {'adapterID': 'cppdbg', 'linesStartAt1': True, 'columnsStartAt1': True,
                                           'pathFormat': 'path', 'supportsRunInTerminalRequest': False})
            launch = adapter.send('launch', config)
            adapter.event('initialized')
            adapter.request('configurationDone')
            adapter.response(launch)
            stopped = adapter.event('stopped')
            threads = adapter.request('threads')['threads']
            self.assertTrue(threads)
            thread = stopped['threadId']
            frames = adapter.request('stackTrace', {'threadId': thread})['stackFrames']
            self.assertTrue(frames)
            adapter.request('evaluate', {'expression': '$pc', 'frameId': frames[0]['id'], 'context': 'watch'})
            points = adapter.request('setFunctionBreakpoints', {'breakpoints': [{'name': 'Plane::one_second_loop'}]})
            self.assertTrue(points['breakpoints'][0]['verified'])
            adapter.request('continue', {'threadId': thread})
            stopped = adapter.event('stopped')
            self.assertEqual(stopped['reason'], 'breakpoint')
            thread = stopped['threadId']
            frames = adapter.request('stackTrace', {'threadId': thread})['stackFrames']
            self.assertIn('one_second_loop', frames[0]['name'])
            scopes = adapter.request('scopes', {'frameId': frames[0]['id']})['scopes']
            self.assertTrue(scopes)
            adapter.request('variables', {'variablesReference': scopes[0]['variablesReference']})
            adapter.request('setFunctionBreakpoints', {'breakpoints': []})
            adapter.request('next', {'threadId': thread, 'granularity': 'instruction'})
            adapter.event('stopped')
            adapter.request('continue', {'threadId': thread})
            adapter.request('pause', {'threadId': thread})
            adapter.event('stopped')
            adapter.request('disconnect', {'terminateDebuggee': False})
            # OpenDebugAD7 acknowledges disconnect then exits; it need not
            # emit a separate terminated event for a client-requested detach.
            self.assertEqual(adapter.process.wait(10), 0)
        finally:
            adapter.close()


if __name__ == '__main__':
    unittest.main()
