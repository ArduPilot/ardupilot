# AP_FLAKE8_CLEAN

from LogAnalyzer import Test, TestResult
from VehicleType import VehicleType

# from ArduCopter/defines.h
AUTOTUNE_INITIALISED = 30
AUTOTUNE_OFF = 31
AUTOTUNE_RESTART = 32
AUTOTUNE_SUCCESS = 33
AUTOTUNE_FAILED = 34
AUTOTUNE_REACHED_LIMIT = 35
AUTOTUNE_PILOT_TESTING = 36
AUTOTUNE_SAVEDGAINS = 37

AUTOTUNE_EVENTS = frozenset(
    [
        AUTOTUNE_INITIALISED,
        AUTOTUNE_OFF,
        AUTOTUNE_RESTART,
        AUTOTUNE_SUCCESS,
        AUTOTUNE_FAILED,
        AUTOTUNE_REACHED_LIMIT,
        AUTOTUNE_PILOT_TESTING,
        AUTOTUNE_SAVEDGAINS,
    ]
)


class TestAutotune(Test):
    '''test for autotune success (copter only)'''

    class AutotuneSession(object):
        def __init__(self, events):
            self.events = events

        @property
        def linestart(self):
            return self.events[0][0]

        @property
        def linestop(self):
            return self.events[-1][0]

        @property
        def success(self):
            return AUTOTUNE_SUCCESS in [i for _, i in self.events]

        @property
        def failure(self):
            return AUTOTUNE_FAILED in [i for _, i in self.events]

        @property
        def limit(self):
            return AUTOTUNE_REACHED_LIMIT in [i for _, i in self.events]

        def __repr__(self):
            return "<AutotuneSession {}-{}>".format(self.linestart, self.linestop)

    def __init__(self):
        Test.__init__(self)
        self.name = "Autotune"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        if logdata.vehicleType != VehicleType.Copter:
            self.result.status = TestResult.StatusType.NA
            return

        for i in ['EV', 'ATDE', 'ATUN']:
            r = False
            if i not in logdata.channels:
                self.result.status = TestResult.StatusType.UNKNOWN
                self.result.statusMessage = "No {} log data".format(i)
                r = True
        if r:
            return

        events = list(filter(lambda x: x[1] in AUTOTUNE_EVENTS, logdata.channels["EV"]["Id"].listData))
        attempts = []

        j = None
        for i in range(0, len(events)):
            line, ev = events[i]
            if ev == AUTOTUNE_INITIALISED:
                if j is not None:
                    attempts.append(TestAutotune.AutotuneSession(events[j:i]))
                j = i

        # last attempt
        if j is not None:
            attempts.append(TestAutotune.AutotuneSession(events[j:]))

        for a in attempts:
            # this should not be necessary!
            def class_from_channel(c):
                members = dict({'__init__': lambda x: setattr(x, i, None) for i in logdata.channels[c]})
                cls = type('Channel__{:s}'.format(c), (object,), members)
                return cls

            # last wins
            if a.success:
                self.result.status = TestResult.StatusType.GOOD
                s = "[+]"
            elif a.failure:
                self.result.status = TestResult.StatusType.FAIL
                s = "[-]"
            else:
                self.result.status = TestResult.StatusType.UNKNOWN
                s = "[?]"

            s += " Autotune {}-{}\n".format(a.linestart, a.linestop)
            self.result.statusMessage += s

            if verbose:
                linenext = a.linestart + 1
                while linenext < a.linestop:
                    try:
                        line = logdata.channels['ATUN']['RateMax'].getNearestValueFwd(linenext)[1]
                        if line > a.linestop:
                            break
                    except ValueError:
                        break
                    atun = class_from_channel('ATUN')()
                    for key in logdata.channels['ATUN']:
                        setattr(atun, key, logdata.channels['ATUN'][key].getNearestValueFwd(linenext)[0])
                    linenext = logdata.channels['ATUN'][key].getNearestValueFwd(linenext)[1] + 1
                    self.result.statusMessage += (
                        "ATUN Axis:{atun.Axis} TuneStep:{atun.TuneStep} RateMin:{atun.RateMin:5.0f}"
                        " RateMax:{atun.RateMax:5.0f} RPGain:{atun.RPGain:1.4f} RDGain:{atun.RDGain:1.4f}"
                        " SPGain:{atun.SPGain:1.1f} (@line:{l})\n"
                    ).format(l=linenext, atun=atun)
                self.result.statusMessage += '\n'
