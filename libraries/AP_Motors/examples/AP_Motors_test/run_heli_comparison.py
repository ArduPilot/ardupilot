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
import csv
from matplotlib import pyplot as plt
from argparse import ArgumentParser


class DataPoints:

    # Instantiate the object and parse the data from the provided file
    # file: path to the csv file to be parsed
    def __init__(self, file):
        self.data = {}
        self.limit_case = []
        self.init_failed = False
        self.seen_header = False

        with open(file, 'r') as csvfile:
            # creating a csv reader object
            csvreader = csv.reader(csvfile)

            # extracting field names through first row
            for row in csvreader:

                if not self.seen_header:
                    # Warn the user if the an init failed message has been found in the file
                    if 'initialisation failed' in row[0]:
                        print('\n%s\n' % row[0])
                        self.init_failed = True
                        break

                    if (row[0] == 'Roll') and (row[1] == 'Pitch') and (row[2] == 'Yaw') and (row[3] == 'Thr'):
                        self.seen_header = True
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

            if not self.seen_header:
                self.init_failed = True

            # Make data immutable
            for field in self.data.keys():
                self.data[field] = tuple(self.data[field])

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

    def get_fields(self):
        return self.data.keys()


frame_class_lookup = {6: 'Single_Heli',
                      11: 'Dual_Heli',
                      13: 'Heli_Quad'}

swash_type_lookup = {0: 'H3',
                     1: 'H1',
                     2: 'H3_140',
                     3: 'H3_120',
                     4: 'H4_90',
                     5: 'H4_45'}


# Run sweep over range of types
def run_sweep(frame_class, swash_type, dir_name):

    # configure and build the test
    os.system('./waf configure --board linux')
    os.system('./waf build --target examples/AP_Motors_test')

    # Run sweep
    for fc in frame_class:
        for swash in swash_type:
            if swash is not None:
                name = 'frame class = %s (%i), swash = %s (%i)' % (frame_class_lookup[fc], fc, swash_type_lookup[swash], swash)
                filename = '%s_%s_motor_test.csv' % (frame_class_lookup[fc], swash_type_lookup[swash])
                swash_cmd = 'swash=%d' % swash

            else:
                name = 'frame class = %s (%i)' % (frame_class_lookup[fc], fc)
                filename = '%s_motor_test.csv' % (frame_class_lookup[fc])
                swash_cmd = ''

            print('Running motors test for %s' % name)
            os.system('./build/linux/examples/AP_Motors_test s frame_class=%d %s > %s/%s' % (fc, swash_cmd, dir_name, filename))
            print('%s complete\n' % name)


if __name__ == '__main__':

    BLUE =  [0, 0, 1]
    RED =   [1, 0, 0]
    BLACK = [0, 0, 0]

    # Build input parser
    parser = ArgumentParser(description='Find logs in which the input string is found in messages')
    parser.add_argument("-H", "--head", type=int, help='number of commits to roll back the head for comparing the work done')
    parser.add_argument("-f", "--frame-class", type=int, dest='frame_class', nargs="+", default=(6, 11), help="list of frame classes to run comparison on. Defaults to test single and dual helis.")
    parser.add_argument("-s", "--swash-type", type=int, dest='swash_type', nargs="+", help="list of swashplate types to run comparison on. Defaults to test all types. Invalid for heli quad")
    parser.add_argument("-c", "--compare", action='store_true', help='Compare only, do not re-run tests')
    parser.add_argument("-p", "--plot", action='store_true', help='Plot comparison results')
    args = parser.parse_args()

    if 13 in args.frame_class:
        if args.swash_type:
            print('Frame %s (%i) does not support swash' % (frame_class_lookup[13], 13))
            quit()
        args.swash_type = [None]

    else:
        if not args.swash_type:
            args.swash_type = (0, 1, 2, 3, 4, 5)

    dir_name = 'motors_comparison'

    if not args.compare:
        # Create the new directory
        if dir_name not in os.listdir('./'):
            os.mkdir(dir_name)

        new_name = dir_name + "/new"
        if "new" not in os.listdir(dir_name):
            os.mkdir(new_name)

        print('\nRunning motor tests with current changes\n')

        # run the test
        run_sweep(args.frame_class, args.swash_type, new_name)

        if args.head:
            # rewind head and repeat test
            if args.head <= 0:
                print('Number of commits to roll back HEAD must be a positive integer value')
                quit()

            # Warn the user that we are about to move things around with git.
            response = input("WARNING: this tool uses git to checkout older commits.  It is safest to do this in a separate test branch.\nDo you wish to continue y/n?:[n]")
            if response.lower() != 'y':
                quit()

            # Rewind the HEAD by the requested number of commits
            original_name = dir_name + "/original"
            if "original" not in os.listdir(dir_name):
                os.mkdir(original_name)

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

            run_sweep(args.frame_class, args.swash_type, original_name)

            # Move back to active branch
            print('Returning to original branch, commit = %s\n' % latest_commit)
            cmd = 'git switch -'
            result = subprocess.run([cmd], shell=True, capture_output=True, text=True)
            print('\n%s\n' % cmd)

            if result.returncode > 0:
                print('WARNING: Could not return head to branch with commit %s. \nError messages:\n%s\n%s' % (latest_commit, result.stdout, result.stderr))

    # Print comparison
    for fc in args.frame_class:
        for sw in args.swash_type:
            frame = frame_class_lookup[fc]
            if sw is not None:
                swash = swash_type_lookup[sw]
                name = frame + ' ' + swash
                filename = '%s_%s_motor_test.csv' % (frame, swash)

            else:
                name = frame
                filename = '%s_motor_test.csv' % (frame)

            print('%s:' % name)

            new_points = DataPoints(os.path.join(dir_name, 'new/%s' % filename))
            old_points = DataPoints(os.path.join(dir_name, 'original/%s' % filename))

            if new_points.init_failed:
                print('\t failed!\n')

            print('\tInputs max change:')
            INPUTS = ['Roll', 'Pitch', 'Yaw', 'Thr']
            input_diff = {}
            for field in INPUTS:
                input_diff[field] = [i-j for i, j in zip(old_points.data[field], new_points.data[field])]
                print('\t\t%s: %f' % (field, max(map(abs, input_diff[field]))))

            # Find number of motors
            num_motors = 0
            while True:
                num_motors += 1
                if 'Mot%i' % (num_motors+1) not in new_points.get_fields():
                    break

            print('\tOutputs max change:')
            output_diff = {}
            for i in range(num_motors):
                field = 'Mot%i' % (i+1)
                output_diff[field] = [i-j for i, j in zip(old_points.data[field], new_points.data[field])]
                print('\t\t%s: %f' % (field, max(map(abs, output_diff[field]))))

            print('\tLimits max change:')
            LIMITS = ['LimR', 'LimP', 'LimY', 'LimThD', 'LimThU']
            limit_diff = {}
            for field in LIMITS:
                limit_diff[field] = [i-j for i, j in zip(old_points.data[field], new_points.data[field])]
                print('\t\t%s: %f' % (field, max(map(abs, limit_diff[field]))))
            print('\n')

            if not args.plot:
                continue

            # Plot comparison
            fig_size = (16, 8)
            fig, ax = plt.subplots(2, 2, figsize=fig_size)
            fig.suptitle('%s Input Diff' % name, fontsize=16)
            ax = ax.flatten()
            for i, field in enumerate(INPUTS):
                ax[i].plot(input_diff[field], color=RED)
                ax[i].set_xlabel('Test Number')
                ax[i].set_ylabel('%s Old - New' % field)
            plt.tight_layout(rect=[0, 0.0, 1, 0.95])

            fig, ax = plt.subplots(2, num_motors, figsize=fig_size)
            fig.suptitle('%s Outputs' % name, fontsize=16)
            for i in range(num_motors):
                field = 'Mot%i' % (i+1)
                ax[0, i].plot(old_points.data[field], color=RED)
                ax[0, i].plot(new_points.data[field], color=BLUE)
                ax[0, i].set_ylabel(field)
                ax[0, i].set_xlabel('Test No')
                ax[1, i].plot(output_diff[field], color=BLACK)
                ax[1, i].set_ylabel('Change in %s' % field)
                ax[1, i].set_xlabel('Test No')
            plt.tight_layout(rect=[0, 0.0, 1, 0.95])

            fig, ax = plt.subplots(2, 5, figsize=fig_size)
            fig.suptitle(name + ' Limits', fontsize=16)
            for i, field in enumerate(LIMITS):
                ax[0, i].plot(old_points.data[field], color=RED)
                ax[0, i].plot(new_points.data[field], color=BLUE)
                ax[0, i].set_ylabel(field)
                ax[0, i].set_xlabel('Test No')
                ax[1, i].plot(limit_diff[field], color=BLACK)
                ax[1, i].set_ylabel('Change in %s' % field)
                ax[1, i].set_xlabel('Test No')
            plt.tight_layout(rect=[0, 0.0, 1, 0.95])

if args.plot:
    plt.show()
