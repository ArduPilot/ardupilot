#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

/*
 * Generic graph implementation using Adjacency matrix
 */
class Graph {

public:
    // To create the initial adjacency matrix
    Graph(uint16_t v);

    // Free all occupied memory
    ~Graph();

    // Function to insert a new edge
    void add_edge(uint16_t start, uint16_t end);

    // returns true if edge is connected
    bool connected(uint16_t start, uint16_t end);

    // returns total number of vertex in the graph
    uint16_t total_nodes() const { return vertex_no; }

private:

    // Adjacency matrix
    bool** adj;

    // Number of vertex
    uint16_t vertex_no;
};