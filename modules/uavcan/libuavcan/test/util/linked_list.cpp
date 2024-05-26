/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include <gtest/gtest.h>
#include <uavcan/util/linked_list.hpp>

struct ListItem : uavcan::LinkedListNode<ListItem>
{
    int value;

    ListItem(int value = 0)
        : value(value)
    { }

    struct GreaterThanComparator
    {
        const int compare_with;

        GreaterThanComparator(int compare_with)
            : compare_with(compare_with)
        { }

        bool operator()(const ListItem* item) const
        {
            return item->value > compare_with;
        }
    };

    void insort(uavcan::LinkedListRoot<ListItem>& root)
    {
        root.insertBefore(this, GreaterThanComparator(value));
    }
};

TEST(LinkedList, Basic)
{
    uavcan::LinkedListRoot<ListItem> root;

    /*
     * Insert/remove
     */
    EXPECT_EQ(0, root.getLength());

    ListItem item1;
    root.insert(&item1);
    root.insert(&item1);         // Insert twice - second will be ignored
    EXPECT_EQ(1, root.getLength());

    root.remove(&item1);
    root.remove(&item1);
    EXPECT_EQ(0, root.getLength());

    ListItem items[3];
    root.insert(&item1);
    root.insert(items + 0);
    root.insert(items + 1);
    root.insert(items + 2);
    EXPECT_EQ(4, root.getLength());

    /*
     * Order persistence
     */
    items[0].value = 10;
    items[1].value = 11;
    items[2].value = 12;
    const int expected_values[] = {12, 11, 10, 0};
    ListItem* node = root.get();
    for (int i = 0; i < 4; i++)
    {
        EXPECT_EQ(expected_values[i], node->value);
        node = node->getNextListNode();
    }

    root.remove(items + 0);
    root.remove(items + 2);
    root.remove(items + 2);
    EXPECT_EQ(2, root.getLength());

    const int expected_values2[] = {11, 0};
    node = root.get();
    for (int i = 0; i < 2; i++)
    {
        EXPECT_EQ(expected_values2[i], node->value);
        node = node->getNextListNode();
    }

    root.insert(items + 2);
    EXPECT_EQ(3, root.getLength());
    EXPECT_EQ(12, root.get()->value);

    /*
     * Emptying
     */
    root.remove(&item1);
    root.remove(items + 0);
    root.remove(items + 1);
    root.remove(items + 2);
    EXPECT_EQ(0, root.getLength());
}

TEST(LinkedList, Sorting)
{
    uavcan::LinkedListRoot<ListItem> root;
    ListItem items[6];
    for (int i = 0; i < 6; i++)
    {
        items[i].value = i;
    }

    EXPECT_EQ(0, root.getLength());

    items[2].insort(root);
    EXPECT_EQ(1, root.getLength());

    items[2].insort(root);              // Making sure the duplicate will be removed
    EXPECT_EQ(1, root.getLength());

    items[3].insort(root);

    items[0].insort(root);

    items[4].insort(root);
    EXPECT_EQ(4, root.getLength());

    items[1].insort(root);
    EXPECT_EQ(5, root.getLength());

    items[1].insort(root);              // Another duplicate
    EXPECT_EQ(5, root.getLength());

    items[5].insort(root);

    EXPECT_EQ(6, root.getLength());

    int prev_val = -100500;
    const ListItem* item = root.get();
    while (item)
    {
        //std::cout << item->value << std::endl;
        EXPECT_LT(prev_val, item->value);
        prev_val = item->value;
        item = item->getNextListNode();
    }
}
