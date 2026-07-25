#
# Copyright (C) 2014-2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#         Ben Dyer <ben_dyer@mac.com>
#

from __future__ import division, absolute_import, print_function, unicode_literals
import decimal


class SourceTimeResolver:
    """
    This class contains logic that recovers absolute value of a remote clock observable via small overflowing
    integer samples.
    For example, consider a remote system that reports timestamps as a 16-bit integer number of milliseconds that
    overflows every 60 seconds (this method is used in SLCAN for example). This class can recover the time difference
    in the remote clock domain between two arbitrary timestamps, even if the timestamp variable overflowed more than
    once between these events.
    """

    def __init__(self, source_clock_overflow_period=None):
        """
        Args:
            source_clock_overflow_period: Overflow period of the remote clock, in seconds.
                                          If not provided, the remote clock is considered to never
                                          overflow (i.e. absolute).
        """
        self.source_clock_overflow_period = \
            decimal.Decimal(source_clock_overflow_period) if source_clock_overflow_period else None

        if self.source_clock_overflow_period is not None and self.source_clock_overflow_period <= 0:
            raise ValueError('source_clock_overflow_period must be positive or None')

        # Internal states
        self._resolved_time = None
        self._prev_source_sample = None
        self._prev_target_sample = None

    def reset(self):
        """
        Resets the internal logic; resolved time will start over.
        """
        self._resolved_time = None
        self._prev_source_sample = None
        self._prev_target_sample = None

    def update(self, source_clock_sample, target_clock_sample):
        """
        Args:
            source_clock_sample: Sample of the source clock, in seconds
            target_clock_sample: Sample of the target clock, in seconds

        Returns: Resolved absolute source clock value
        """
        if self._resolved_time is None or self.source_clock_overflow_period is None:
            self._resolved_time = decimal.Decimal(source_clock_sample)
            self._prev_source_sample = source_clock_sample
            self._prev_target_sample = target_clock_sample
        else:
            # Time between updates in the target clock domain
            tgt_delta = target_clock_sample - self._prev_target_sample
            self._prev_target_sample = target_clock_sample
            assert tgt_delta >= 0

            # Time between updates in the source clock domain
            src_delta = source_clock_sample - self._prev_source_sample
            self._prev_source_sample = source_clock_sample

            # Using the target clock we can resolve the integer ambiguity (number of overflows)
            full_cycles = int(round((tgt_delta - src_delta) / float(self.source_clock_overflow_period), 0))

            # Updating the source clock now; in two steps, in order to avoid error accumulation in floats
            self._resolved_time += decimal.Decimal(full_cycles * self.source_clock_overflow_period)
            self._resolved_time += decimal.Decimal(src_delta)

        return self._resolved_time


class TimestampEstimator:
    """
    Based on "A Passive Solution to the Sensor Synchronization Problem" [Edwin Olson 2010]
        https://april.eecs.umich.edu/pdfs/olson2010.pdf
    """

    DEFAULT_MAX_DRIFT_PPM = 200
    DEFAULT_MAX_PHASE_ERROR_TO_RESYNC = 1.

    def __init__(self,
                 max_rate_error=None,
                 source_clock_overflow_period=None,
                 fixed_delay=None,
                 max_phase_error_to_resync=None):
        """
        Args:
            max_rate_error:                 The max drift parameter must be not lower than maximum relative clock
                                            drift in PPM. If the max relative drift is guaranteed to be lower,
                                            reducing this value will improve estimation. The default covers vast
                                            majority of low-cost (and up) crystal oscillators.
            source_clock_overflow_period:   How often the source clocks wraps over, in seconds.
                                            For example, for SLCAN this value is 60 seconds.
                                            If not provided, the source clock is considered to never wrap over.
            fixed_delay:                    This value will be unconditionally added to the delay estimations.
                                            Represented in seconds. Default is zero.
                                            For USB-interfaced sources it should be safe to use as much as 100 usec.
            max_phase_error_to_resync:      When this value is exceeded, the estimator will start over.
                                            Defaults to a large value.
        """
        self.max_rate_error = float(max_rate_error or (self.DEFAULT_MAX_DRIFT_PPM / 1e6))
        self.fixed_delay = fixed_delay or 0
        self.max_phase_error_to_resync = max_phase_error_to_resync or self.DEFAULT_MAX_PHASE_ERROR_TO_RESYNC

        if self.max_rate_error < 0:
            raise ValueError('max_rate_error must be non-negative')

        if self.fixed_delay < 0:
            raise ValueError('fixed_delay must be non-negative')

        if self.max_phase_error_to_resync <= 0:
            raise ValueError('max_phase_error_to_resync must be positive')

        # This is used to recover absolute source time
        self._source_time_resolver = SourceTimeResolver(source_clock_overflow_period=source_clock_overflow_period)

        # Refer to the paper for explanations
        self._p = None
        self._q = None

        # Statistics
        self._estimated_delay = 0.0
        self._resync_count = 0

    def update(self, source_clock_sample, target_clock_sample):
        """
        Args:
            source_clock_sample:    E.g. value received from the source system, in seconds
            target_clock_sample:    E.g. target time sampled when the data arrived to the local system, in seconds
        Returns: Event timestamp converted to the target time domain.
        """
        pi = float(self._source_time_resolver.update(source_clock_sample, target_clock_sample))
        qi = target_clock_sample

        # Initialization
        if self._p is None:
            self._p = pi
            self._q = qi

        # Sync error - refer to the reference implementation of the algorithm
        self._estimated_delay = abs((pi - self._p) - (qi - self._q))

        # Resynchronization (discarding known state)
        if self._estimated_delay > self.max_phase_error_to_resync:
            self._source_time_resolver.reset()
            self._resync_count += 1
            self._p = pi = float(self._source_time_resolver.update(source_clock_sample, target_clock_sample))
            self._q = qi

        # Offset options
        assert pi >= self._p
        offset = self._p - self._q - self.max_rate_error * (pi - self._p) - self.fixed_delay
        new_offset = pi - qi - self.fixed_delay

        # Updating p/q if the new offset is lower by magnitude
        if new_offset >= offset:
            offset = new_offset
            self._p = pi
            self._q = qi

        ti = pi - offset

        return ti

    @property
    def estimated_delay(self):
        """Estimated delay, updated in the last call to update()"""
        return self._estimated_delay

    @property
    def resync_count(self):
        return self._resync_count


if __name__ == '__main__':
    # noinspection PyPackageRequirements
    import matplotlib.pyplot as plt
    # noinspection PyPackageRequirements
    import numpy
    import time

    if 1:
        estimator = TimestampEstimator()
        print(estimator.update(.0, 1000.0))
        print(estimator.update(.1, 1000.1))
        print(estimator.update(.2, 1000.1))  # Repeat
        print(estimator.update(.3, 1000.1))  # Repeat
        print(estimator.update(.4, 1000.2))
        print(estimator.update(.5, 1000.3))

    if 1:
        # Conversion from Real to Monotonic
        estimator = TimestampEstimator(max_rate_error=1e-5,
                                       fixed_delay=1e-6,
                                       max_phase_error_to_resync=1e-2)
        print('Initial mono to real:', time.time() - time.monotonic())
        while True:
            mono = time.monotonic()
            real = time.time()
            est_real = estimator.update(mono, real)
            mono_to_real_offset = est_real - mono
            print(mono_to_real_offset)
            time.sleep(1)

    max_rate_error = None
    source_clock_range = 10
    delay_min = 0.0001
    delay_max = 0.02
    num_samples = 200

    x = range(num_samples)
    delays = numpy.random.uniform(delay_min, delay_max, size=num_samples)

    estimator = TimestampEstimator(max_rate_error=max_rate_error, fixed_delay=delay_min,
                                   source_clock_overflow_period=source_clock_range)

    source_clocks = []
    estimated_times = []
    offset_errors = []
    estimated_delays = []
    for i, delay in enumerate(delays):
        source_clock = i
        source_clocks.append(source_clock)
        target_clock = i + delay

        estimated_time = estimator.update(source_clock % source_clock_range, target_clock)
        estimated_times.append(estimated_time)
        offset_errors.append(estimated_time - source_clock)
        estimated_delays.append(estimator.estimated_delay)

    fig = plt.figure()

    ax1 = fig.add_subplot(211)
    ax1.plot(x, numpy.array(delays) * 1e3)
    ax1.plot(x, numpy.array(offset_errors) * 1e3)

    ax2 = fig.add_subplot(212)
    ax2.plot(x, (numpy.array(estimated_times) - numpy.array(source_clocks)) * 1e3)

    plt.show()
