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

#include "MutexChecker.h"
#include <AP_HAL/AP_HAL.h>

#define MC_ASSERT(x) if (!(x)) { AP_HAL::panic("MC_ASSERT: %d", __LINE__); }
#define CHECK_AND_EXPAND(x, ret) do { \
                                        if (x ## _space <= x ## _length) { \
                                            if (!x.expand_to_hold(x ## _space * 2)) return ret; \
                                            x ## _space *= 2; \
                                        } \
                                    } while(0)
MutexNode* MutexChecker::find_or_add(void* mtx)
{
    // find mtx in the graph
    uint16_t index;
    for (index = 0; index < graph_length; index++) {
        if (graph[index].mtx == mtx) {
            break;
        }
    }
    if (index == graph_length) {
        // mtx is not in the graph, add it
        CHECK_AND_EXPAND(graph, nullptr);
        graph[index].mtx = mtx;
        graph[index].rank = index;
        graph[index].visited = false;
        graph_length++;
    }
    return &graph[index];
}

// mtx_x is the child
// mtx_y is the parent
bool MutexChecker::insert(void* mtx_x, void* mtx_y)
{
    // find mtx_x in the graph
    MutexNode* x = find_or_add(mtx_x);
    if (x == nullptr) {
        MC_ASSERT(false);
        return false;
    }
    // find mtx_y in the graph
    MutexNode* y = find_or_add(mtx_y);
    if (y == nullptr) {
        MC_ASSERT(false);
        return false;
    }

    if (x==y) {
        // mtx_x and mtx_y are the same mutex
        return true;
    }

    // add mtx_y as parent of mtx_x
    if (!x->add_in(y)) {
        MC_ASSERT(false);
        return false;
    }
    // add mtx_x as child of mtx_y
    if (!y->add_out(x)) {
        MC_ASSERT(false);
        return false;
    }

    if (x->rank >= y->rank) {
        return true;
    }

    // reset visited states
    for (uint16_t i = 0; i < graph_length; i++) {
        graph[i].visited = false;
    }

    // we need to reorganize the graph
    // find all the parents of mtx_y
    if (!forward_dfs(y, x->rank)) {
        if (loop_found) {
            MC_ASSERT(false);
            return false;
        }
        MC_ASSERT(false);
        return false;
    }

    // reset visited states
    for (uint16_t i = 0; i < graph_length; i++) {
        graph[i].visited = false;
    }
    if (!backward_dfs(x, y->rank)) {
        MC_ASSERT(false);
        return false;
    }
    if (!reorder_and_merge()) {
        MC_ASSERT(false);
        return false;
    }
    return true;
}

bool MutexChecker::forward_dfs(MutexNode* start_node, uint16_t upper_bound)
{
    stack[stack_length++] = start_node;

    while (stack_length) {
        MutexNode* node = stack[--stack_length];
        if (node->visited) {
            continue;
        }
        node->visited = true;

        CHECK_AND_EXPAND(fw_list, false);
        fw_list[fw_list_length++] = node;
        if (node->rank == upper_bound) {
            return false;
        }
        for (uint16_t i=0; i<node->out_cnt; i++) {
            MutexNode* child = node->out[i];
            if (child->visited) {
                continue;
            }
            CHECK_AND_EXPAND(stack, false);
            if (child->rank < upper_bound) {
                stack[stack_length++] = child;
            }
        }
    }
    return true;
}

bool MutexChecker::backward_dfs(MutexNode* start_node, uint16_t lower_bound)
{
    stack[stack_length++] = start_node;

    while (stack_length) {
        MutexNode* node = stack[--stack_length];
        if (node->visited) {
            continue;
        }
        node->visited = true;

        CHECK_AND_EXPAND(bw_list, false);
        bw_list[bw_list_length++] = node;
        if (node->rank == lower_bound) {
            return false;
        }
        for (uint16_t i=0; i<node->in_cnt; i++) {
            MutexNode* parent = node->in[i];
            if (parent->visited) {
                continue;
            }

            CHECK_AND_EXPAND(stack, false);
            if (parent->rank > lower_bound) {
                stack[stack_length++] = parent;
            }
        }
    }
    return true;
}

void MutexChecker::sort_updated_rank()
{
    // sort the updated_rank
    for (uint16_t i=0; i<updated_rank_length; i++) {
        for (uint16_t j=i+1; j<updated_rank_length; j++) {
            if (updated_rank[i] > updated_rank[j]) {
                uint16_t tmp = updated_rank[i];
                updated_rank[i] = updated_rank[j];
                updated_rank[j] = tmp;
            }
        }
    }
}

bool MutexChecker::reorder_and_merge()
{
    // get the ranks of bw_list node into updated_rank
    for (uint16_t i=0; i<bw_list_length; i++) {
        MutexNode* node = bw_list[i];
        CHECK_AND_EXPAND(updated_rank, false);
        updated_rank[updated_rank_length++] = node->rank;
    }

    // get the ranks of fw_list node into updated_rank
    for (uint16_t i=0; i<fw_list_length; i++) {
        MutexNode* node = fw_list[i];
        CHECK_AND_EXPAND(updated_rank, false);
        updated_rank[updated_rank_length++] = node->rank;
    }

    // sort the updated_rank
    sort_updated_rank();

    // update the rank of the nodes in bw_list and then fw_list
    uint16_t rank_index = 0;
    for (uint16_t i = 0; i < bw_list_length; i++) {
        MutexNode* node = bw_list[i];
        node->rank = updated_rank[rank_index++];
    }
    for (uint16_t i = 0; i < fw_list_length; i++) {
        MutexNode* node = fw_list[i];
        node->rank = updated_rank[rank_index++];
    }
    return true;
}
