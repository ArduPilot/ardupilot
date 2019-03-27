/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_gtest.h>

#include <pthread.h>
#include <unistd.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Linux/Thread.h>
#include <AP_HAL_Linux/PollerThread.h>

using namespace Linux;

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

class TestThread1 : public Thread {
public:
    TestThread1() : Thread(nullptr) { }

    int n_loop = 0;

protected:
    bool _run() override {
        n_loop = 1;
        return true;
    }
};

TEST(LinuxThread, override_run)
{
    TestThread1 thr;
    EXPECT_TRUE(thr.start(nullptr, 0, 0));

    while (thr.n_loop == 0) {
        usleep(10000);
    }

    EXPECT_EQ(thr.n_loop, 1);
}

class TestThread2 : public Thread {
public:
    TestThread2() : Thread{FUNCTOR_BIND_MEMBER(&TestThread2::_task, void)} { }

    int n_loop = 0;

protected:
    void _task() {
        n_loop = 1;
    }
};

TEST(LinuxThread, run_task)
{
    TestThread2 thr;
    EXPECT_TRUE(thr.start(nullptr, 0, 0));

    while (thr.n_loop == 0) {
        usleep(10000);
    }

    EXPECT_EQ(thr.n_loop, 1);
}

TEST(LinuxThread, poller_thread)
{
    PollerThread thr;
    EXPECT_TRUE(thr.start(nullptr, 0, 0));

    while (!thr.is_started()) {
        usleep(1000);
    }

    usleep(10000);

    EXPECT_TRUE(thr.stop());
    EXPECT_TRUE(thr.join());
}

class TestPeriodicThread1 : public PeriodicThread {
public:
    TestPeriodicThread1() : PeriodicThread{FUNCTOR_BIND_MEMBER(&TestPeriodicThread1::_task, void)} { }
protected:
    void _task() { }
};

TEST(LinuxThread, periodic_thread)
{
    TestPeriodicThread1 thr;
    EXPECT_TRUE(thr.set_rate(1000));
    EXPECT_TRUE(thr.start(nullptr, 0, 0));

    while (!thr.is_started()) {
        usleep(1000);
    }

    // this must fail as the thread already started
    EXPECT_FALSE(thr.set_rate(10));

    usleep(10000);

    EXPECT_TRUE(thr.stop());
    EXPECT_TRUE(thr.join());
}

AP_GTEST_MAIN()
