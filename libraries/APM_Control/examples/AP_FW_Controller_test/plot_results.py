# Plot the results of a controller test run
# For example run test with:
# ./build/linux/examples/AP_FW_Controller_test > results.csv
# The plot with:
# python3 libraries/APM_Control/examples/AP_FW_Controller_test/plot_results.py results.csv

# flake8: noqa

import csv
from argparse import ArgumentParser
from collections import defaultdict
from matplotlib import pyplot as plt

if __name__ == '__main__':

    parser = ArgumentParser(description='Plot results of controller test run')
    parser.add_argument('filename', help='csv file to read')
    args = parser.parse_args()

    with open(args.filename) as csv_file:
        csv_reader = csv.DictReader(csv_file)

        # Skip First few lines
        next(csv_file, None)
        next(csv_file, None)
        next(csv_file, None)
        next(csv_file, None)
        next(csv_file, None)
        next(csv_file, None)
        next(csv_file, None)
        next(csv_file, None)

        csv_data = [row for row in csv_reader]

    # Reshape data from list of dicts to dict of lists
    data = defaultdict(list)
    for i in csv_data:
        for key, value in i.items():
            data[key.strip()].append(float(value))

    # Make some plots!

    fig, ax = plt.subplots(2)
    fig.suptitle('Input => output')
    ax[0].plot(data['Time (s)'], data['angle error'])
    ax[0].set_ylabel('input angle error (deg)')
    ax[1].plot(data['Time (s)'], data['output'])
    ax[1].set_ylabel('output servo position')
    ax[1].set_xlabel('Time (s)')

    fig, ax = plt.subplots(2)
    fig.suptitle('Rate controller')
    ax[0].plot(data['Time (s)'], data['PID target'], label = "target")
    ax[0].plot(data['Time (s)'], data['PID actual'], label = "actual")
    ax[0].legend()
    ax[0].set_ylabel('rate target and actual')
    ax[1].plot(data['Time (s)'], data['error'])
    ax[1].set_ylabel('rate error')
    ax[1].set_xlabel('Time (s)')

    fig = plt.figure()
    plt.title('Rate controller PID components')
    plt.plot(data['Time (s)'], data['P'], label = "P")
    plt.plot(data['Time (s)'], data['I'], label = "I")
    plt.plot(data['Time (s)'], data['D'], label = "D")
    plt.plot(data['Time (s)'], data['FF'], label = "FF")
    plt.plot(data['Time (s)'], data['DFF'], label = "DFF")
    plt.legend()
    plt.xlabel('Time (s)')

    fig, ax = plt.subplots(2)
    fig.suptitle('Rate controller')
    ax[0].plot(data['Time (s)'], data['Slew rate'])
    ax[0].set_ylabel('Slew rate')
    ax[1].plot(data['Time (s)'], data['Dmod'])
    ax[1].set_ylabel('Dmod')
    ax[1].set_xlabel('Time (s)')

    fig = plt.figure()
    plt.title('Rate controller PID flags')
    plt.plot(data['Time (s)'], data['limit'], label = "limit")
    plt.plot(data['Time (s)'], data['PD limit'], label = "PD limit")
    plt.plot(data['Time (s)'], data['reset'], label = "reset")
    plt.plot(data['Time (s)'], data['I term set'], label = "I term set")
    plt.legend()
    plt.xlabel('Time (s)')

    plt.show()
