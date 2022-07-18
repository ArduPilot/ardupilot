# AP_FLAKE8_CLEAN

from LogAnalyzer import Test, TestResult
from VehicleType import VehicleType


class TestBalanceTwist(Test):
    '''test for badly unbalanced copter, including yaw twist'''

    def __init__(self):
        Test.__init__(self)
        self.name = "Motor Balance"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        if logdata.vehicleType != VehicleType.Copter:
            self.result.status = TestResult.StatusType.NA
            return

        self.result.status = TestResult.StatusType.UNKNOWN
        if "RCOU" not in logdata.channels:
            return

        ch = []

        for i in range(8):
            for prefix in "Chan", "Ch", "C":
                if prefix + repr((i + 1)) in logdata.channels["RCOU"]:
                    ch.append(map(lambda x: x[1], logdata.channels["RCOU"][prefix + repr((i + 1))].listData))

        ch = zip(*ch)
        num_channels = 0
        ch = list(ch)
        for i in range(len(ch)):
            ch[i] = list(filter(lambda x: (x > 0 and x < 3000), ch[i]))
            if num_channels < len(ch[i]):
                num_channels = len(ch[i])

        if logdata.frame:
            num_channels = logdata.num_motor_channels()

        if num_channels < 2:
            return

        try:
            min_throttle = (
                logdata.parameters["RC3_MIN"]
                + logdata.parameters["THR_MIN"]
                / (logdata.parameters["RC3_MAX"] - logdata.parameters["RC3_MIN"])
                / 1000.0
            )
        except KeyError:
            min_throttle = (
                logdata.parameters["MOT_PWM_MIN"]
                / (logdata.parameters["MOT_PWM_MAX"] - logdata.parameters["RC3_MIN"])
                / 1000.0
            )

        ch = list(filter(lambda x: sum(x) / num_channels > min_throttle, ch))

        if len(ch) == 0:
            return

        avg_sum = 0
        avg_ch = []
        for i in range(num_channels):
            avg = list(map(lambda x: x[i], ch))
            avg = sum(avg) / len(avg)
            avg_ch.append(avg)
            avg_sum += avg
        avg_all = avg_sum / num_channels

        self.result.statusMessage = (
            "Motor channel averages = %s\nAverage motor output = %.0f\nDifference between min and max motor averages = %.0f"
            % (str(avg_ch), avg_all, abs(min(avg_ch) - max(avg_ch)))
        )

        self.result.status = TestResult.StatusType.GOOD

        if abs(min(avg_ch) - max(avg_ch)) > 75:
            self.result.status = TestResult.StatusType.WARN
        if abs(min(avg_ch) - max(avg_ch)) > 150:
            self.result.status = TestResult.StatusType.FAIL
