from LogAnalyzer import Test,TestResult
import DataflashLog
from math import sqrt


class TestIMUMatch(Test):
    '''test for empty or near-empty logs'''

    def __init__(self):
        Test.__init__(self)
        self.name = "IMU Mismatch"

    def run(self, logdata, verbose):

        #tuning parameters:
        warn_threshold = .75
        fail_threshold = 1.5
        filter_tc = 5.0

        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        if ("IMU" in logdata.channels) and (not "IMU2" in logdata.channels):
            self.result.status = TestResult.StatusType.NA
            self.result.statusMessage = "No IMU2"
            return

        if (not "IMU" in logdata.channels) or (not "IMU2" in logdata.channels):
            self.result.status = TestResult.StatusType.UNKNOWN
            self.result.statusMessage = "No IMU log data"
            return

        imu1 = logdata.channels["IMU"]
        imu2 = logdata.channels["IMU2"]

        timeLabel = None
        for i in 'TimeMS','TimeUS','Time':
            if i in logdata.channels["GPS"]:
                timeLabel = i
                break
        imu1_timems = imu1[timeLabel].listData
        imu1_accx = imu1["AccX"].listData
        imu1_accy = imu1["AccY"].listData
        imu1_accz = imu1["AccZ"].listData

        imu2_timems = imu2[timeLabel].listData
        imu2_accx = imu2["AccX"].listData
        imu2_accy = imu2["AccY"].listData
        imu2_accz = imu2["AccZ"].listData

        imu_multiplier = 1.0E-3
        if timeLabel == 'TimeUS':
            imu_multiplier = 1.0E-6

        imu1 = []
        imu2 = []

        for i in range(len(imu1_timems)):
            imu1.append({ 't': imu1_timems[i][1]*imu_multiplier, 'x': imu1_accx[i][1], 'y': imu1_accy[i][1], 'z': imu1_accz[i][1]})

        for i in range(len(imu2_timems)):
            imu2.append({ 't': imu2_timems[i][1]*imu_multiplier, 'x': imu2_accx[i][1], 'y': imu2_accy[i][1], 'z': imu2_accz[i][1]})

        imu1.sort(key=lambda x: x['t'])
        imu2.sort(key=lambda x: x['t'])

        imu2_index = 0

        last_t = None

        xdiff_filtered = 0
        ydiff_filtered = 0
        zdiff_filtered = 0
        max_diff_filtered = 0

        for i in range(len(imu1)):
            #find closest imu2 value
            t = imu1[i]['t']
            dt = 0 if last_t is None else t-last_t
            dt=min(dt,.1)

            next_imu2 = None
            for i in range(imu2_index,len(imu2)):
                next_imu2 = imu2[i]
                imu2_index=i
                if next_imu2['t'] >= t:
                    break
            prev_imu2 = imu2[imu2_index-1]
            closest_imu2 = next_imu2 if abs(next_imu2['t']-t)<abs(prev_imu2['t']-t) else prev_imu2

            xdiff = imu1[i]['x']-closest_imu2['x']
            ydiff = imu1[i]['y']-closest_imu2['y']
            zdiff = imu1[i]['z']-closest_imu2['z']

            xdiff_filtered += (xdiff-xdiff_filtered)*dt/filter_tc
            ydiff_filtered += (ydiff-ydiff_filtered)*dt/filter_tc
            zdiff_filtered += (zdiff-zdiff_filtered)*dt/filter_tc

            diff_filtered = math.sqrt(xdiff_filtered**2+ydiff_filtered**2+zdiff_filtered**2)
            max_diff_filtered = max(max_diff_filtered,diff_filtered)
            #print max_diff_filtered
            last_t = t

        if max_diff_filtered > fail_threshold:
            self.result.statusMessage = "Check vibration or accelerometer calibration. (Mismatch: %.2f, WARN: %.2f, FAIL: %.2f)" % (max_diff_filtered,warn_threshold,fail_threshold)
            self.result.status = TestResult.StatusType.FAIL
        elif max_diff_filtered > warn_threshold:
            self.result.statusMessage = "Check vibration or accelerometer calibration. (Mismatch: %.2f, WARN: %.2f, FAIL: %.2f)" % (max_diff_filtered,warn_threshold,fail_threshold)
            self.result.status = TestResult.StatusType.WARN
        else:
            self.result.statusMessage = "(Mismatch: %.2f, WARN: %.2f, FAIL: %.2f)" % (max_diff_filtered,warn_threshold, fail_threshold)


