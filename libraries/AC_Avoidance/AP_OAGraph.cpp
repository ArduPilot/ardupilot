/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_OAGraph.h"

// Function to fill the empty adjacency matrix
Graph::Graph(uint16_t v)
{
    vertex_no = v;
    adj = new bool*[v];
    for (uint16_t row = 0; row < v; row++) {
        adj[row] = new bool[v];
        for (uint16_t column = 0; column < v; column++) {
            adj[row][column] = false;
        }
    }
}


// Function to add an edge to the graph
void Graph::add_edge(uint16_t start, uint16_t e)
{
    // Considering a bidirectional edge
    adj[start][e] = true;
    adj[e][start] = true;
}

// returns true if edge is connected
bool Graph::connected(uint16_t start, uint16_t end)
{
    if (adj[start][end] || adj[end][start]) {
        return true;
    }
    return false;
}

// Free all occupied memory.
Graph::~Graph()
{
    // delete all the allocated memory for this graph
    for (uint16_t row = 0; row < vertex_no; row++) {
        delete [] adj[row];
    }
    delete [] adj;
}
