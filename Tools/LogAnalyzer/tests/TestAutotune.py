from LogAnalyzer import Test,TestResult
import DataflashLog


class TestAutotune(Test):
    '''test for autotune success (copter only)'''

    def __init__(self):
        Test.__init__(self)
        self.name = "Autotune"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        if logdata.vehicleType != "ArduCopter":
            self.result.status = TestResult.StatusType.NA
            return

        for i in ['EV','ATDE','ATUN']:
            r = False
            if not i in logdata.channels:
                self.result.status = TestResult.StatusType.UNKNOWN
                self.result.statusMessage = "No {} log data".format(i)
                r = True
        if r:
            return

        for line,ev in logdata.channels["EV"]["Id"].listData:
            if ev != 30: # Autotune Start
                continue
            nextline = startline = line
            nextev = -1
            while nextev not in(35,34):
                try:
                    (nextev,nextline) = logdata.channels["EV"]["Id"].getNearestValueFwd(nextline+1)
                except:
                    break

                if nextev not in(35,34): # autotune is still running
                    continue

                if nextev == 34: # autotune failed
                    self.result.status = TestResult.StatusType.FAIL
                    s = "[-]"
                else:
                    self.result.status = TestResult.StatusType.GOOD
                    s = "[+]"

                # this should not be necessary!
                def class_from_channel(c):
                    members = dict({'__init__':lambda x: setattr(x,i,None) for i in logdata.channels[c]})
                    cls = type(\
                               'Channel__{:s}'.format(c),
                               (object,),
                               members
                               )
                    return cls

                atde = class_from_channel('ATDE')()
                for key in logdata.channels['ATDE']:
                    setattr(atde, key, logdata.channels['ATDE'][key].getNearestValueBack(nextline)[0])

                atun = class_from_channel('ATUN')()
                for key in logdata.channels['ATUN']:
                    setattr(atun, key, logdata.channels['ATUN'][key].getNearestValueBack(nextline)[0])
                self.result.statusMessage += '{s} ATDE Angle:{atde.Angle} Rate:{atde.Rate} ATUN RPGain:{atun.RPGain} RDGain:{atun.RDGain} SPGain:{atun.SPGain} (@line:{l})\n'.format(l=nextline,s=s,atde=atde, atun=atun)

            if nextev not in(35,34):
                self.result.statusMessage += "incomplete autotune attempt started @{l}\n".format(l=startline)




