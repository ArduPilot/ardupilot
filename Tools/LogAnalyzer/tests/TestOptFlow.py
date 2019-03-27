from LogAnalyzer import Test,TestResult
import DataflashLog

from math import sqrt
import numpy as np
import matplotlib.pyplot as plt

class TestFlow(Test):
    '''test optical flow sensor scale factor calibration'''
    #
    # Use the following procedure to log the calibration  data. is assumed that the optical flow sensor has been
    # correctly aligned, is focussed and the test is performed over a textured surface with adequate lighting.
    # Note that the strobing effect from non incandescent artifical lighting can produce poor optical flow measurements.
    #
    # 1) Set LOG_DISARMED and FLOW_ENABLE to 1 and verify that ATT and OF messages are being logged onboard
    # 2) Place on level ground, apply power and wait for EKF to complete attitude alignment
    # 3) Keeping the copter level, lift it to shoulder height and rock between +-20 and +-30 degrees
    #    in roll about an axis that passes through the flow sensor lens assembly. The time taken to rotate from
    #    maximum left roll to maximum right roll should be about 1 second.
    # 4) Repeat 3) about the pitch axis
    # 5) Holding the copter level, lower it to the ground and remove power
    # 6) Transfer the logfile from the sdcard.
    # 7) Open a terminal and cd to the ardupilot/Tools/LogAnalyzer directory
    # 8) Enter to run the analysis 'python LogAnalyzer.py <log file name including full path>'
    # 9) Check the OpticalFlow test status printed to the screen. The analysis plots are saved to
    #    flow_calibration.pdf and the recommended scale factors to flow_calibration.param

    def __init__(self):
        Test.__init__(self)
        self.name = "OpticalFlow"

    def run(self, logdata, verbose):
        self.result = TestResult()
        self.result.status = TestResult.StatusType.GOOD

        def FAIL():
            self.result.status = TestResult.StatusType.FAIL

        def WARN():
            if self.result.status != TestResult.StatusType.FAIL:
                self.result.status = TestResult.StatusType.WARN

        try:
            # tuning parameters used by the algorithm
            tilt_threshold = 15 # roll and pitch threshold used to start and stop calibration (deg)
            quality_threshold = 124 # minimum flow quality required for data to be used by the curve fit (N/A)
            min_rate_threshold = 0.0 # if the gyro rate is less than this, the data will not be used by the curve fit (rad/sec)
            max_rate_threshold = 2.0 # if the gyro rate is greter than this, the data will not be used by the curve fit (rad/sec)
            param_std_threshold = 5.0 # maximum allowable 1-std uncertainty in scaling parameter (scale factor * 1000)
            param_abs_threshold = 200 # max/min allowable scale factor parameter. Values of FLOW_FXSCALER and FLOW_FYSCALER outside the range of +-param_abs_threshold indicate a sensor configuration problem.
            min_num_points = 100 # minimum number of points required for a curve fit - this is necessary, but not sufficient condition - the standard deviation estimate of the fit gradient is also important.

            # get the existing scale parameters
            flow_fxscaler = logdata.parameters["FLOW_FXSCALER"]
            flow_fyscaler = logdata.parameters["FLOW_FYSCALER"]

            # load required optical flow data
            if "OF" in logdata.channels:
                flowX = np.zeros(len(logdata.channels["OF"]["flowX"].listData))
                for i in range(len(logdata.channels["OF"]["flowX"].listData)):
                    (line, flowX[i])  = logdata.channels["OF"]["flowX"].listData[i]

                bodyX = np.zeros(len(logdata.channels["OF"]["bodyX"].listData))
                for i in range(len(logdata.channels["OF"]["bodyX"].listData)):
                    (line, bodyX[i])  = logdata.channels["OF"]["bodyX"].listData[i]

                flowY = np.zeros(len(logdata.channels["OF"]["flowY"].listData))
                for i in range(len(logdata.channels["OF"]["flowY"].listData)):
                    (line, flowY[i])  = logdata.channels["OF"]["flowY"].listData[i]

                bodyY = np.zeros(len(logdata.channels["OF"]["bodyY"].listData))
                for i in range(len(logdata.channels["OF"]["bodyY"].listData)):
                    (line, bodyY[i])  = logdata.channels["OF"]["bodyY"].listData[i]

                flow_time_us = np.zeros(len(logdata.channels["OF"]["TimeUS"].listData))
                for i in range(len(logdata.channels["OF"]["TimeUS"].listData)):
                    (line, flow_time_us[i])  = logdata.channels["OF"]["TimeUS"].listData[i]

                flow_qual = np.zeros(len(logdata.channels["OF"]["Qual"].listData))
                for i in range(len(logdata.channels["OF"]["Qual"].listData)):
                    (line, flow_qual[i])  = logdata.channels["OF"]["Qual"].listData[i]

            else:
                FAIL()
                self.result.statusMessage = "FAIL: no optical flow data\n"
                return

            # load required attitude data
            if "ATT" in logdata.channels:
                Roll = np.zeros(len(logdata.channels["ATT"]["Roll"].listData))
                for i in range(len(logdata.channels["ATT"]["Roll"].listData)):
                    (line, Roll[i])  = logdata.channels["ATT"]["Roll"].listData[i]

                Pitch = np.zeros(len(logdata.channels["ATT"]["Pitch"].listData))
                for i in range(len(logdata.channels["ATT"]["Pitch"].listData)):
                    (line, Pitch[i])  = logdata.channels["ATT"]["Pitch"].listData[i]

                att_time_us = np.zeros(len(logdata.channels["ATT"]["TimeUS"].listData))
                for i in range(len(logdata.channels["ATT"]["TimeUS"].listData)):
                    (line, att_time_us[i])  = logdata.channels["ATT"]["TimeUS"].listData[i]

            else:
                FAIL()
                self.result.statusMessage = "FAIL: no attitude data\n"
                return

            # calculate the start time for the roll calibration
            startTime = int(0)
            startRollIndex = int(0)
            for i in range(len(Roll)):
                if abs(Roll[i]) > tilt_threshold:
                    startTime = att_time_us[i]
                    break
            for i in range(len(flow_time_us)):
                if flow_time_us[i] > startTime:
                    startRollIndex = i
                    break

            # calculate the end time for the roll calibration
            endTime = int(0)
            endRollIndex = int(0)
            for i in range(len(Roll)-1,-1,-1):
                if abs(Roll[i]) > tilt_threshold:
                    endTime = att_time_us[i]
                    break
            for i in range(len(flow_time_us)-1,-1,-1):
                if flow_time_us[i] < endTime:
                    endRollIndex = i
                    break

            # check we have enough roll data points
            if (endRollIndex - startRollIndex <= min_num_points):
                FAIL()
                self.result.statusMessage = "FAIL: insufficient roll data pointsa\n"
                return

            # resample roll test data excluding data before first movement and after last movement
            # also exclude data where there is insufficient angular rate
            flowX_resampled = []
            bodyX_resampled = []
            flowX_time_us_resampled = []
            for i in range(len(Roll)):
                if (i >= startRollIndex) and (i <= endRollIndex) and (abs(bodyX[i]) > min_rate_threshold) and (abs(bodyX[i]) < max_rate_threshold) and (flow_qual[i] > quality_threshold):
                    flowX_resampled.append(flowX[i])
                    bodyX_resampled.append(bodyX[i])
                    flowX_time_us_resampled.append(flow_time_us[i])

            # calculate the start time for the pitch calibration
            startTime = 0
            startPitchIndex = int(0)
            for i in range(len(Pitch)):
                if abs(Pitch[i]) > tilt_threshold:
                    startTime = att_time_us[i]
                    break
            for i in range(len(flow_time_us)):
                if flow_time_us[i] > startTime:
                    startPitchIndex = i
                    break

            # calculate the end time for the pitch calibration
            endTime = 0
            endPitchIndex = int(0)
            for i in range(len(Pitch)-1,-1,-1):
                if abs(Pitch[i]) > tilt_threshold:
                    endTime = att_time_us[i]
                    break
            for i in range(len(flow_time_us)-1,-1,-1):
                if flow_time_us[i] < endTime:
                    endPitchIndex = i
                    break

            # check we have enough pitch data points
            if (endPitchIndex - startPitchIndex <= min_num_points):
                FAIL()
                self.result.statusMessage = "FAIL: insufficient pitch data pointsa\n"
                return

            # resample pitch test data excluding data before first movement and after last movement
            # also exclude data where there is insufficient or too much angular rate
            flowY_resampled = []
            bodyY_resampled = []
            flowY_time_us_resampled = []
            for i in range(len(Roll)):
                if (i >= startPitchIndex) and (i <= endPitchIndex) and (abs(bodyY[i]) > min_rate_threshold) and (abs(bodyY[i]) < max_rate_threshold) and (flow_qual[i] > quality_threshold):
                    flowY_resampled.append(flowY[i])
                    bodyY_resampled.append(bodyY[i])
                    flowY_time_us_resampled.append(flow_time_us[i])

            # fit a straight line to the flow vs body rate data and calculate the scale factor parameter required to achieve a slope of 1
            coef_flow_x , cov_x = np.polyfit(bodyX_resampled,flowX_resampled,1,rcond=None, full=False, w=None, cov=True)
            coef_flow_y , cov_y = np.polyfit(bodyY_resampled,flowY_resampled,1,rcond=None, full=False, w=None, cov=True)

            # taking the exisiting scale factor parameters into account, calculate the parameter values reequired to achieve a unity slope
            flow_fxscaler_new = int(1000 * (((1 + 0.001 * float(flow_fxscaler))/coef_flow_x[0] - 1)))
            flow_fyscaler_new = int(1000 * (((1 + 0.001 * float(flow_fyscaler))/coef_flow_y[0] - 1)))

            # Do a sanity check on the scale factor variance
            if sqrt(cov_x[0][0]) > param_std_threshold or sqrt(cov_y[0][0]) > param_std_threshold:
                FAIL()
                self.result.statusMessage = "FAIL: inaccurate fit - poor quality or insufficient data\nFLOW_FXSCALER 1STD = %u\nFLOW_FYSCALER 1STD = %u\n" % (round(1000*sqrt(cov_x[0][0])),round(1000*sqrt(cov_y[0][0])))

            # Do a sanity check on the scale factors
            if abs(flow_fxscaler_new) > param_abs_threshold or abs(flow_fyscaler_new) > param_abs_threshold:
                FAIL()
                self.result.statusMessage = "FAIL: required scale factors are excessive\nFLOW_FXSCALER=%i\nFLOW_FYSCALER=%i\n" % (flow_fxscaler,flow_fyscaler)

            # display recommended scale factors
            self.result.statusMessage = "Set FLOW_FXSCALER to %i\nSet FLOW_FYSCALER to %i\n\nCal plots saved to flow_calibration.pdf\nCal parameters saved to flow_calibration.param\n\nFLOW_FXSCALER 1STD = %u\nFLOW_FYSCALER 1STD = %u\n" % (flow_fxscaler_new,flow_fyscaler_new,round(1000*sqrt(cov_x[0][0])),round(1000*sqrt(cov_y[0][0])))

            # calculate fit display data
            body_rate_display = [-max_rate_threshold,max_rate_threshold]
            fit_coef_x = np.poly1d(coef_flow_x)
            flowX_display = fit_coef_x(body_rate_display)
            fit_coef_y = np.poly1d(coef_flow_y)
            flowY_display = fit_coef_y(body_rate_display)

            # plot and save calibration test points to PDF
            from matplotlib.backends.backend_pdf import PdfPages
            output_plot_filename = "flow_calibration.pdf"
            pp = PdfPages(output_plot_filename)

            plt.figure(1,figsize=(20,13))
            plt.subplot(2,1,1)
            plt.plot(bodyX_resampled,flowX_resampled,'b', linestyle=' ', marker='o',label="test points")
            plt.plot(body_rate_display,flowX_display,'r',linewidth=2.5,label="linear fit")
            plt.title('X axis flow rate vs gyro rate')
            plt.ylabel('flow rate (rad/s)')
            plt.xlabel('gyro rate (rad/sec)')
            plt.grid()
            plt.legend(loc='upper left')

            # draw plots
            plt.subplot(2,1,2)
            plt.plot(bodyY_resampled,flowY_resampled,'b', linestyle=' ', marker='o',label="test points")
            plt.plot(body_rate_display,flowY_display,'r',linewidth=2.5,label="linear fit")
            plt.title('Y axis flow rate vs gyro rate')
            plt.ylabel('flow rate (rad/s)')
            plt.xlabel('gyro rate (rad/sec)')
            plt.grid()
            plt.legend(loc='upper left')

            pp.savefig()

            plt.figure(2,figsize=(20,13))
            plt.subplot(2,1,1)
            plt.plot(flow_time_us,flowX,'b',label="flow rate - all")
            plt.plot(flow_time_us,bodyX,'r',label="gyro rate - all")
            plt.plot(flowX_time_us_resampled,flowX_resampled,'c', linestyle=' ', marker='o',label="flow rate - used")
            plt.plot(flowX_time_us_resampled,bodyX_resampled,'m', linestyle=' ', marker='o',label="gyro rate - used")
            plt.title('X axis flow and body rate vs time')
            plt.ylabel('rate (rad/s)')
            plt.xlabel('time (usec)')
            plt.grid()
            plt.legend(loc='upper left')

            # draw plots
            plt.subplot(2,1,2)
            plt.plot(flow_time_us,flowY,'b',label="flow rate - all")
            plt.plot(flow_time_us,bodyY,'r',label="gyro rate - all")
            plt.plot(flowY_time_us_resampled,flowY_resampled,'c', linestyle=' ', marker='o',label="flow rate - used")
            plt.plot(flowY_time_us_resampled,bodyY_resampled,'m', linestyle=' ', marker='o',label="gyro rate - used")
            plt.title('Y axis flow and body rate vs time')
            plt.ylabel('rate (rad/s)')
            plt.xlabel('time (usec)')
            plt.grid()
            plt.legend(loc='upper left')

            pp.savefig()

            # close the pdf file
            pp.close()

            # close all figures
            plt.close("all")

            # write correction parameters to file
            test_results_filename = "flow_calibration.param"
            file = open(test_results_filename,"w")
            file.write("FLOW_FXSCALER"+" "+str(flow_fxscaler_new)+"\n")
            file.write("FLOW_FYSCALER"+" "+str(flow_fyscaler_new)+"\n")
            file.close()

        except KeyError as e:
            self.result.status = TestResult.StatusType.FAIL
            self.result.statusMessage = str(e) + ' not found'










