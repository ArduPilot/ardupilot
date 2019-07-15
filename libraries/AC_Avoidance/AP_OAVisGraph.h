#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/AP_ExpandingArray.h>
#include <AP_HAL/AP_HAL.h>

/*
 * Visibility graph used by Dijkstra's algorithm for path planning around fence, stay-out zones and moving obstacles
 */
class AP_OAVisGraph {
public:
    AP_OAVisGraph();

    /* Do not allow copies */
    AP_OAVisGraph(const AP_OAVisGraph &other) = delete;
    AP_OAVisGraph &operator=(const AP_OAVisGraph&) = delete;

    // types of items held in graph
    enum OAType : uint8_t {
        OATYPE_SOURCE = 0,
        OATYPE_DESTINATION,
        OATYPE_INTERMEDIATE_POINT,
    };

    // support up to 255 items of each type
    typedef uint8_t oaid_num;

    // id for uniquely identifying objects held in visibility graphs and paths
    class OAItemID {
    public:
        OAType id_type;
        oaid_num id_num;
        bool operator ==(const OAItemID &i) const { return ((id_type == i.id_type) && (id_num == i.id_num)); }
    };

    struct VisGraphItem {
        OAItemID id1;       // first item's id
        OAItemID id2;       // second item's id
        float distance_cm;  // distance between the items
    };

    // clear all elements from graph
    void clear() { _num_items = 0; }

    // get number of items in visibility graph table
    uint16_t num_items() const { return _num_items; }

    // add item to visiblity graph, returns true on success, false if graph is full
    bool add_item(const OAItemID &id1, const OAItemID &id2, float distance_cm);

    // allow accessing graph as an array, 0 indexed
    // Note: no protection against out-of-bounds accesses so use with num_items()
    const VisGraphItem& operator[](uint16_t i) const { return _items[i]; }

private:

    AP_ExpandingArray<VisGraphItem> _items;
    uint16_t _num_items;
};
