/*
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
 *
 * Code by Siddharth Bharat Purohit, CubePilot Pty Ltd
 */
#include <AP_gtest.h>
#include <AP_HAL/utility/MutexChecker.h>
#include <AP_HAL/AP_HAL.h>

TEST(MutexCheckerTest, FindOrAdd)
{
    // we don't care about semaphore lock or unlock, pass empty functions
    static uint16_t lock_cnt = 0;
    static uint16_t unlock_cnt = 0;
    MutexChecker mc(nullptr, [](void *){lock_cnt++;}, [](void *){unlock_cnt++;});
    void *mtx = (void *)0x1234;
    MutexNode *node = mc.find_or_add(mtx);
    EXPECT_EQ(node->mtx, mtx);
    EXPECT_EQ(node->rank, 0);
    EXPECT_FALSE(node->visited);
}

