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
#include <AP_HAL/HAL.h>
#include <AP_HAL/utility/MutexChecker.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(MutexCheckerTest, FindOrAdd)
{
    // we don't care about semaphore lock or unlock, pass empty functions
    static uint16_t lock_cnt = 0;
    static uint16_t unlock_cnt = 0;
    MutexChecker *mc = new MutexChecker(nullptr, [](void *){lock_cnt++;}, [](void *){unlock_cnt++;});
    uint32_t *mtx = (uint32_t *)0x1234;
    for (uint8_t i=0; i<20; i++) {
        MutexNode *node = mc->find_or_add(&mtx[i]);
        EXPECT_EQ(node->mtx, &mtx[i]);
        EXPECT_EQ(node->rank, i);
        EXPECT_FALSE(node->visited);
    }
    for (uint8_t i=0; i<20; i++) {
        MutexNode *node = mc->find_or_add(&mtx[i]);
        EXPECT_EQ(node->mtx, &mtx[i]);
        EXPECT_EQ(node->rank, i);
        EXPECT_FALSE(node->visited);
    }
    EXPECT_EQ(lock_cnt, 0);
    EXPECT_EQ(unlock_cnt, 0);
}

AP_GTEST_MAIN()
