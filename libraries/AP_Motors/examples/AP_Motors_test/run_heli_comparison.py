# Script that automatically runs multipoint output comparison for all AP_Motors_Heli frames
# To prove equivalence when doing restructuring PR's
#
# Run from "ardupilot" directory
# Run the command below. The first argument is the number of commits to rewind the HEAD to get the "old"
# comparison point.  This should rewind back past all of the commits you have added
#
# If you just want to run a specific frame class, (e.g. dual heli = 11) then add the -f argument giving the
# frame class number (e.g. -f 11) to only run that frame class.  Default is to run all heli frame classes
#
# command to run:
# ---------------
# python3 libraries/AP_Motors/examples/AP_Motors_test/run_heli_comparison.py <INESRT No COMMITS TO REWIND>
#
# You may have to run "./waf distclean" if failing to build

import os
import subprocess
import shutil
import csv
from matplotlib import pyplot as plt
from argparse import ArgumentParser
import time

# ==============================================================================
class DataPoints:

    HEADER_LINE = 3

    # --------------------------------------------------------------------------
    # Instantiate the object and parse the data from the provided file
    # file: path to the csv file to be parsed
    def __init__(self, file):
        self.data = {}
        self.limit_case = []
        self.init_failed = False

        with open(file, 'r') as csvfile:
            # creating a csv reader object
            csvreader = csv.reader(csvfile)

            # extracting field names through first row
            line_num = 0
            for row in csvreader:
                line_num += 1

                if line_num < self.HEADER_LINE:
                    # Warn the user if the an init failed message has been found in the file
                    if 'initialisation failed' in row[0]:
                        print('\n%s\n' % row[0])
                        self.init_failed = True
                    continue

                elif line_num == self.HEADER_LINE:
                    # init all dict entries based on the header entries
                    for field in row:
                        self.data[field] = []

                else:
                    # stow all of the data
                    case_is_limited = False
                    for field, data in zip(self.data.keys(), row):
                        self.data[field].append(float(data))

                        # Keep track of all cases where a limit flag is set
                        if ('lim' in field.lower()) and (float(data) > 0.5):
                            case_is_limited = True
                    self.limit_case.append(case_is_limited)

            # Make data immutable
            for field in self.data.keys():
                self.data[field] = tuple(self.data[field])
    # --------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    # get the data from a given field
    # field: dict index, name of field data to be returned
    # lim_tf: limit bool, return limit cases or not
    def get_data(self, field, lim_tf):
        if field not in self.data.keys():
            raise Exception('%s is not a valid data field' % field)

        ret = []
        for data, flag in zip(self.data[field], self.limit_case):
            if (flag == lim_tf):
                ret.append(data)
        return ret
    # --------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    def get_fields(self):
        return self.data.keys()
    # --------------------------------------------------------------------------

# ==============================================================================

frame_class_lookup = {6:'Single_Heli', 11:'Dual_Heli', 13:'Heli_Quad'}


# ==============================================================================
if __name__ == '__main__':

    BLUE = [0,0,1]
    RED = [1,0,0]
    BLACK = [0,0,0]

    dir_name = 'motors_comparison'

    # Build input parser
    parser = ArgumentParser(description='Find logs in which the input string is found in messages')
    parser.add_argument("head", type=int, help='number of commits to roll back the head for comparing the work done')
    parser.add_argument("-f","--frame-class", type=int, dest='frame_class', nargs="+", default=(6,11,13), help="list of frame classes to run comparison on. Defaults to 6 single heli.")
    args = parser.parse_args()

    # Warn the user that we are about to move things around with git.
    response = input("WARNING: this tool uses git to checkout older commits.  It is safest to do this in a separate test branch.\nDo you wish to continue y/n?:[n]")
    if response.lower() != 'y':
        quit()

    if not args.head:
        print('Number of commits to roll back HEAD must be provided to run comparison. Add --help for more info.')
        quit()
    if args.head <= 0:
        print('Number of commits to roll back HEAD must be a positive integer value')
        quit()

    # If we have already run this test, delete the old data
    if dir_name in os.listdir('./'):
        shutil.rmtree(dir_name)

    # Create the new directory
    os.mkdir(dir_name)
 
    # configure and build the test
    os.system('./waf configure --board linux')
    os.system('./waf build --target examples/AP_Motors_test')

    print('\nRunning motor tests with current changes\n')

    # run the test
    for fc in args.frame_class:
        filename = 'new_%s_motor_test.csv' % frame_class_lookup[fc]
        os.system('./build/linux/examples/AP_Motors_test s > %s frame_class=%d' % (filename,fc))

        # move the csv to the directory for later comparison
        shutil.move(filename, os.path.join(dir_name, filename))

        print('Frame class = %s complete\n' % frame_class_lookup[fc])

    # Rewind the HEAD by the requested number of commits
    cmd = 'git log -%d --format=format:"%%H"' % (args.head+1)
    result = subprocess.run([cmd], shell=True, capture_output=True, text=True)
    git_history = result.stdout.split('\n')
    latest_commit = git_history[0]
    base_commit = git_history[-1]

    print('Rewinding head by %d commits to: %s\n' % (args.head, base_commit))

    # Checkout to a detached head to test the old code before improvements
    cmd = 'git checkout %s' % base_commit
    result = subprocess.run([cmd], shell=True, capture_output=True, text=True)
    print('\n%s\n' % cmd)

    if result.returncode > 0:
        print('ERROR: Could not rewind HEAD.  Exited with error:\n%s\n%s' % (result.stdout, result.stderr))
        quit()

    # Rebuild 
    os.system('./waf clean')
    os.system('./waf build --target examples/AP_Motors_test')

    # Run motors test for "old" comparison point
    for fc in args.frame_class:
        print('Running motors test for frame class = %s complete\n' % frame_class_lookup[fc])

        filename = 'original_%s_motor_test.csv' % frame_class_lookup[fc]
        os.system('./build/linux/examples/AP_Motors_test s > %s frame_class=%d' % (filename,fc))

        # move the csv to the directory for later comparison
        shutil.move(filename, os.path.join(dir_name, filename))

        print('Frame class = %s, test complete\n' % frame_class_lookup[fc])

    # Move back to active branch
    print('Returning to original branch, commit = %s\n' % latest_commit)
    cmd = 'git switch -' 
    result = subprocess.run([cmd], shell=True, capture_output=True, text=True)
    print('\n%s\n' % cmd)

    if result.returncode > 0:
        print('WARNING: Could not return head to branch with commit %s. \nError messages:\n%s\n%s' % (latest_commit, result.stdout, result.stderr))

    new_points = {}
    old_points = {}
    for fc in args.frame_class:
        filename = '%s_motor_test.csv' % frame_class_lookup[fc]
        new_points[frame_class_lookup[fc]] = DataPoints(os.path.join(dir_name, 'new_%s' % filename))
        old_points[frame_class_lookup[fc]] = DataPoints(os.path.join(dir_name, 'original_%s' % filename))

    # Plot all of the points for correlation comparison
    for fc in args.frame_class:
        frame = frame_class_lookup[fc]
        fig_size = (16, 8)

        # Ensure we didn't get an init fail before proceeding
        if new_points[frame].init_failed:
            # Create plot explaining issue to user, as the earlier cmd line warning may have been lost
            fig, ax = plt.subplots(1, 1, figsize=fig_size)
            fig.suptitle('%s: INIT FAILED' % frame, fontsize=16)
            continue

        # Plot inputs
        fig, ax = plt.subplots(2, 2, figsize=fig_size)
        fig.suptitle('%s Input Diff' % frame, fontsize=16)
        ax = ax.flatten()

        plot_index = 0
        for field in ['Roll','Pitch','Yaw','Thr']:
            diff = [i-j for i,j in zip(old_points[frame].data[field], new_points[frame].data[field])]
            ax[plot_index].plot(diff, color=RED)
            ax[plot_index].set_xlabel('Test Number')
            ax[plot_index].set_ylabel('%s Old - New' % field)
            plot_index += 1
        plt.tight_layout(rect=[0, 0.0, 1, 0.95])

        # Find number of motors
        num_motors = 0
        while True:
            num_motors += 1
            if 'Mot%i' % (num_motors+1) not in new_points[frame].get_fields():
                break

        # Plot outputs
        fig, ax = plt.subplots(2, num_motors, figsize=fig_size)
        fig.suptitle('%s Outputs' % frame, fontsize=16)
        for i in range(num_motors):
            field = 'Mot%i' % (i+1)
            ax[0,i].plot(old_points[frame].data[field], color=RED)
            ax[0,i].plot(new_points[frame].data[field], color=BLUE)
            ax[0,i].set_ylabel(field)
            ax[0,i].set_xlabel('Test No')

            diff = [i-j for i,j in zip(old_points[frame].data[field], new_points[frame].data[field])]
            ax[1,i].plot(diff, color=BLACK)
            ax[1,i].set_ylabel('Change in %s' % field)
            ax[1,i].set_xlabel('Test No')
        plt.tight_layout(rect=[0, 0.0, 1, 0.95])

        # Plot limits
        fig, ax = plt.subplots(2, 5, figsize=fig_size)
        fig.suptitle(frame + ' Limits', fontsize=16)
        for i, field in enumerate(['LimR','LimP','LimY','LimThD','LimThU']):
            ax[0,i].plot(old_points[frame].data[field], color=RED)
            ax[0,i].plot(new_points[frame].data[field], color=BLUE)
            ax[0,i].set_ylabel(field)
            ax[0,i].set_xlabel('Test No')

            diff = [i-j for i,j in zip(old_points[frame].data[field], new_points[frame].data[field])]
            ax[1,i].plot(diff, color=BLACK)
            ax[1,i].set_ylabel('Change in %s' % field)
            ax[1,i].set_xlabel('Test No')
        plt.tight_layout(rect=[0, 0.0, 1, 0.95])

    print('*** Complete ***')
    plt.show()
