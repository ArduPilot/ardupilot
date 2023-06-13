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

MutexNode* MutexChecker::find_or_add(void* mtx)
{
    // find mtx in the graph
    uint16_t index;
    for (index = 0; index < graph_cnt; index++) {
        if (_graph[index].mtx == mtx) {
            break;
        }
    }
    if (index == graph_cnt) {
        // mtx is not in the graph, add it
        if (graph_cnt == available_graph_cnt) {
            if (!_graph.expand_to_hold(graph_cnt * 2)) return nullptr; // double the size
        }
        _graph[index].mtx = mtx;
        _graph[index].rank = index;
        _graph[index].visited = false;
        graph_cnt++;
    }
    return &_graph[index];
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
    for (uint16_t i = 0; i < graph_cnt; i++) {
        _graph[i].visited = false;
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
    for (uint16_t i = 0; i < graph_cnt; i++) {
        _graph[i].visited = false;
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
        if (fw_list_length == fw_list_space) {
            if (!fw_list.expand_to_hold(fw_list_space * 2)) {
                return false;
            }
            fw_list_space *= 2;
        }
        fw_list[fw_list_length++] = node;
        if (node->rank == upper_bound) {
            return false;
        }
        for (uint16_t i=0; i<node->out_cnt; i++) {
            MutexNode* child = node->out[i];
            if (child->visited) {
                continue;
            }
            if (stack_length == stack_space) {
                if (!stack.expand_to_hold(stack_space * 2)) {
                    return false;
                }
                stack_space *= 2;
            }
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
        if (bw_list_length == bw_list_space) {
            if (!bw_list.expand_to_hold(bw_list_space * 2)) {
                return false;
            }
            bw_list_space *= 2;
        }
        bw_list[bw_list_length++] = node;
        if (node->rank == lower_bound) {
            return false;
        }
        for (uint16_t i=0; i<node->in_cnt; i++) {
            MutexNode* parent = node->in[i];
            if (parent->visited) {
                continue;
            }
            if (stack_length == stack_space) {
                if (!stack.expand_to_hold(stack_space * 2)) {
                    return false;
                }
                stack_space *= 2;
            }
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
        if (updated_rank_length == updated_rank_space) {
            if (!updated_rank.expand_to_hold(updated_rank_space * 2)) {
                return false;
            }
            updated_rank_space *= 2;
        }
        updated_rank[updated_rank_length++] = node->rank;
    }

    // get the ranks of fw_list node into updated_rank
    for (uint16_t i=0; i<fw_list_length; i++) {
        MutexNode* node = fw_list[i];
        if (updated_rank_length == updated_rank_space) {
            if (!updated_rank.expand_to_hold(updated_rank_space * 2)) {
                return false;
            }
            updated_rank_space *= 2;
        }
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
