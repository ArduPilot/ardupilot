#!/usr/bin/env python
'''
decode an watchdog message

/Tools/scripts/decode_watchdog.py "WDOG, 2641424, -3, 0, 0, 0, 0, 0, 0, 122, 3, 0, 181, 4196355, 135203219, SPI1"

AP_FLAKE8_CLEAN
'''

import re
import sys
import optparse
from collections import OrderedDict

import decode_ICSR


class DecodeWatchDog(object):

    class Component(object):
        def __init__(self, value):
            self.value = value

        def prefix(self):
            m = re.match(".*Component([A-Z]+)", str(type(self)))
            return m.group(1)

        def decode(self):
            return "?????"

        def string_value(self):
            return str(self.value)

        def print_decoded(self):
            print("%5s %25s: %12s: %s" % (
                self.prefix(),
                self.expansion(),
                self.string_value(),
                self.decode()
            ))

    class ComponentT(Component):

        def expansion(self):
            return "Scheduler Task"

        def decode(self):
            if int(self.value) == -3:
                return "Waiting for sample"
            if int(self.value) == -1:
                return "Pre-loop"
            if int(self.value) == -2:
                return "Fast loop"
            return self.value

    class ComponentSL(Component):

        def expansion(self):
            return "Semaphore Line"

        def decode(self):
            if int(self.value) == 0:
                return "Not waiting on semaphore"
            return self.value

    class ComponentFL(Component):
        def expansion(self):
            return "Fault Line"

    class ComponentFT(Component):

        def expansion(self):
            return "Fault Type"

        def decode(self):
            x = int(self.value)
            # this list taken from AP_HAL_ChibiOS/system.cpp
            fault_types = {
                1: "Reset",
                2: "NMI",
                3: "HardFault",
                4: "MemManage",
                5: "BusFault",
                6: "UsageFault",
            }
            if x in fault_types:
                return fault_types[x]
            return super(DecodeWatchDog.ComponentFT, self).decode()

    class ComponentFA(Component):

        def expansion(self):
            return "Fault Address"

        def string_value(self):
            return hex(int(self.value, 16))

    class ComponentFTP(Component):

        def expansion(self):
            return "Fault Thread Priority"

    class ComponentFLR(Component):

        def expansion(self):
            return "Fault Long Return Address"  # ?? FIXME: is this really what LR stands for?

        def string_value(self):
            return "0x" + self.value

    class ComponentFICSR(Component):

        def expansion(self):
            return "Fault ICS Register"   # ?? FIXME: expand further

        def string_value(self):
            return hex(int(self.value, 16))

        def decode(self):
            return "[Below]"

        def print_decoded(self):
            super(DecodeWatchDog.ComponentFICSR, self).print_decoded()
            decoder = decode_ICSR.DecodeICSR()
            text = decoder.string(int(self.value))
            sys.stdout.write(re.sub("^", "        ", text, flags=re.M))

    class ComponentMM(Component):

        def expansion(self):
            return "MAVLink Message"

        def decode(self):
            if int(self.value) == 0:
                return "[None]"
            return super(DecodeWatchDog.ComponentMM, self).decode()

    class ComponentMC(Component):

        def expansion(self):
            return "MAVLink Command"

        def decode(self):
            if int(self.value) == 0:
                return "[None]"
            return super(DecodeWatchDog.ComponentMC, self).decode()

    class ComponentIE(Component):

        def expansion(self):
            return "Internal Error Mask"

    class ComponentIEHex(ComponentIE):

        def expansion(self):
            return "Internal Error Mask"

        def string_value(self):
            return hex(int(self.value, 16))

    class ComponentIEC(Component):

        def expansion(self):
            return "Internal Error Count"

        def decode(self):
            return self.value

    class ComponentIEL(Component):

        def expansion(self):
            return "Internal Error Line"

        def decode(self):
            return self.value

    class ComponentTN(Component):

        def expansion(self):
            return "Thread name"

    def __init__(self):
        self.components = OrderedDict()
        self.components["T"] = DecodeWatchDog.ComponentT
        self.components["SL"] = DecodeWatchDog.ComponentSL
        self.components["FL"] = DecodeWatchDog.ComponentFL
        self.components["FT"] = DecodeWatchDog.ComponentFT
        self.components["FA"] = DecodeWatchDog.ComponentFA
        self.components["FTP"] = DecodeWatchDog.ComponentFTP
        self.components["FLR"] = DecodeWatchDog.ComponentFLR
        self.components["FICSR"] = DecodeWatchDog.ComponentFICSR
        self.components["MM"] = DecodeWatchDog.ComponentMM
        self.components["MC"] = DecodeWatchDog.ComponentMC
        self.components["IE"] = DecodeWatchDog.ComponentIEHex
        self.components["IEC"] = DecodeWatchDog.ComponentIEC
        self.components["TN"] = DecodeWatchDog.ComponentTN

        self.df_components = {}
        self.df_components["Task"] = DecodeWatchDog.ComponentT
        self.df_components["Tsk"] = DecodeWatchDog.ComponentT
        self.df_components["IErr"] = DecodeWatchDog.ComponentIE
        self.df_components["IE"] = DecodeWatchDog.ComponentIE
        self.df_components["IEC"] = DecodeWatchDog.ComponentIEC
        self.df_components["IEL"] = DecodeWatchDog.ComponentIEL
        self.df_components["MavMsg"] = DecodeWatchDog.ComponentMM
        self.df_components["MvMsg"] = DecodeWatchDog.ComponentMM
        self.df_components["MvCmd"] = DecodeWatchDog.ComponentMC
        self.df_components["SemLine"] = DecodeWatchDog.ComponentSL
        self.df_components["SmLn"] = DecodeWatchDog.ComponentSL
        self.df_components["FL"] = DecodeWatchDog.ComponentFL
        self.df_components["FT"] = DecodeWatchDog.ComponentFT
        self.df_components["FA"] = DecodeWatchDog.ComponentFA
        self.df_components["FP"] = DecodeWatchDog.ComponentFTP
        self.df_components["LR"] = DecodeWatchDog.ComponentFLR
        self.df_components["ICSR"] = DecodeWatchDog.ComponentFICSR
        self.df_components["TN"] = DecodeWatchDog.ComponentTN

    def run(self, text):

        # see if the supplied string is a statustext message:
        re_string = "(?:APM: )?WDG:"
        for component in self.components.keys():
            re_string += " %s(?P<%s>[^ ]+)" % (component, component)

        # print("string: %s" % text)
        # print("re_string: %s" % re_string)

        wdg_re = re.compile(re_string)

        m = wdg_re.match(text)
        if m is not None:
            comp = []
            for group in m.groupdict():
                comp.append(self.components[group](m.group(group)))
            for c in comp:
                c.print_decoded()
            return

        # not a statustext message; see if it a WDOG dataflash message
        df_re = re.compile("WDOG {(.*)}")
        m = df_re.match(text)
        if m is not None:
            pairs = m.group(1).split(",")
            for pair in pairs:
                (name, value) = pair.split(":")
                name = name.strip()
                if name == "TimeUS":
                    continue
                value = value.strip()
#                print("(%s)=(%s)" % (name, value))
                if name in ["LR", "FICSR", "FA"]:
                    value = int(value, 10)
                    value = hex(value)
                    value = value[2:]
                if name not in self.df_components:
                    raise KeyError(name)
                self.df_components[name](value).print_decoded()
            return

        # not a statustext message and not a mavlogdump dump of a WDOG
        # dataflash message.  See if it is a .log-style CSV line
        log_re = re.compile(r"WDOG, (\d+), ([-\d]+), ([-\d]+), ([-\d]+), ([-\d]+), ([-\d]+), ([-\d]+), ([-\d]+), ([-\d]+), "
                            r"([-\d]+), ([-\d]+), ([-\d]+), ([-\d]+), ([-\d]+), (\w+)")
        column_names = "TimeUS,Tsk,IE,IEC,IEL,MvMsg,MvCmd,SmLn,FL,FT,FA,FP,ICSR,LR,TN"
        cols = column_names.split(",")
        m = log_re.match(text)
        if m is not None:
            for i in range(0, len(cols)):
                name = cols[i]
                if name == 'TimeUS':
                    continue
                value = m.group(i+1)
                # convert some things from base10 to hex:
                if name in ["LR", "FICSR", "FA"]:
                    value = int(value, 10)
                    value = hex(value)
                    value = value[2:]
                if name not in self.df_components:
                    raise KeyError(name)
                self.df_components[name](value).print_decoded()
            return

        raise ValueError("Text not recognised")

    # 2020-06-10 17:20:08.45: WDOG {TimeUS : 949568, Task : -2, IErr : 0, IErrCnt : 0, MavMsg : 0, MavCmd : 0, SemLine : 0, FL : 100, FT : 3, FA : 404947019, FP : 183, ICSR : 4196355}  # NOQA

    # APM: WDG: T-3 SL0 FL122 FT3 FA0 FTP177 FLR80CBB35 FICSR4196355 MM0 MC0 IE67108864 IEC12353 TN:rcin

    # FMT, 254, 47, WDOG, QbIHHHHHHHIBIIn, TimeUS,Tsk,IE,IEC,IEL,MvMsg,MvCmd,SmLn,FL,FT,FA,FP,ICSR,LR,TN
    # WDOG, 2641424, -3, 0, 0, 0, 0, 0, 0, 122, 3, 0, 181, 4196355, 135203219, SPI1


if __name__ == '__main__':

    parser = optparse.OptionParser(__file__)

    opts, args = parser.parse_args()

    if len(args) == 0:
        print("Usage: %s" % parser.usage)
        sys.exit(0)

    text = args[0]

    decoder = DecodeWatchDog()
    decoder.run(text)
