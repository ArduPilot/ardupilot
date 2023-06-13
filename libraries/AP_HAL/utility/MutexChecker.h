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

#include <stdint.h>
#include <AP_Common/AP_ExpandingArray.h>

struct MutexNode {
    void* mtx;
    bool visited;
    uint16_t rank;

    AP_ExpandingArray<MutexNode*> out{8};
    uint16_t out_cnt;
    uint16_t available_out_cnt = 8;
    bool add_out(MutexNode* node) {
        if (out_cnt == available_out_cnt) {
            if (!out.expand_to_hold(out_cnt * 2)) return false; // double the size
        }
        out[out_cnt++] = node;
        return true;
    }

    AP_ExpandingArray<MutexNode*> in{8};
    uint16_t in_cnt;
    uint16_t available_in_cnt = 8;
    bool add_in(MutexNode* node) {
        if (in_cnt == available_in_cnt) {
            if (!in.expand_to_hold(in_cnt * 2)) return false; // double the size
        }
        in[in_cnt++] = node;
        return true;
    }
};

// Sorted Topological graph of mutexes
class MutexChecker {
    typedef void (*lock_fn)(void*);
    typedef void (*unlock_fn)(void*);
public:
    // Raii class to lock and unlock mutex
    class Lock {
    public:
        Lock(MutexChecker &_mc) : mc(_mc) {
            mc._lock(mc._self_mtx);
        }
        ~Lock() {
            mc._unlock(mc._self_mtx);
        }
    private:
        MutexChecker &mc;
    };
#define MC_SELF_LOCK() Lock _lock(_self_mtx)

    MutexChecker(void* mtx, lock_fn lock, unlock_fn unlock) : 
    _self_mtx(mtx),
    _lock(lock),
    _unlock(unlock) {}

    // Insert an edge to the graph, returns true if the graph is acyclic
    // or false if the graph has a cycle or out of memory
    // also updates mtx_x and mtx_y to point to the mutexes in the graph
    // with cycle.
    bool insert(void* mtx_x, void* mtx_y);

    MutexNode* find_or_add(void* mtx);
    bool forward_dfs(MutexNode* node, uint16_t upper_bound);
    bool backward_dfs(MutexNode* node, uint16_t lower_bound);
    void sort_updated_rank();
    bool reorder_and_merge();

private:
    bool loop_found;

    void* _self_mtx;
    lock_fn _lock;
    unlock_fn _unlock;
    uint16_t graph_cnt;
    uint16_t available_graph_cnt = 8;
    AP_ExpandingArray<MutexNode> _graph{8};


    AP_ExpandingArray<MutexNode*> fw_list{8};
    uint16_t fw_list_length;
    uint16_t fw_list_space = 8;

    AP_ExpandingArray<MutexNode*> bw_list{8};
    uint16_t bw_list_length;
    uint16_t bw_list_space = 8;

    AP_ExpandingArray<MutexNode*> stack{8};
    uint16_t stack_length;
    uint16_t stack_space = 8;

    AP_ExpandingArray<uint32_t> updated_rank{8};
    uint16_t updated_rank_length = 0;
    uint16_t updated_rank_space = 8;
};
