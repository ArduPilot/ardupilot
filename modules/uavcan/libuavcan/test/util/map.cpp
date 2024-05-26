/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#if __GNUC__
# pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#endif

#include <string>
#include <cstdio>
#include <memory>
#include <gtest/gtest.h>
#include <uavcan/util/map.hpp>


/*
 * TODO: This one test has been temporarily disabled because it is not compatible with newer versions of libstdc++
 * that ship with newer versions of GCC. The problem is that std::string has become too large to fit into a 64-byte
 * large memory block. This should be fixed in the future.
 */
#if 0
static std::string toString(long x)
{
    char buf[80];
    std::snprintf(buf, sizeof(buf), "%li", x);
    return std::string(buf);
}

static bool oddValuePredicate(const std::string& key, const std::string& value)
{
    EXPECT_FALSE(key.empty());
    EXPECT_FALSE(value.empty());
    const int num = atoi(value.c_str());
    return num & 1;
}

struct KeyFindPredicate
{
    const std::string target;
    KeyFindPredicate(std::string target) : target(target) { }
    bool operator()(const std::string& key, const std::string&) const { return key == target; }
};

struct ValueFindPredicate
{
    const std::string target;
    ValueFindPredicate(std::string target) : target(target) { }
    bool operator()(const std::string&, const std::string& value) const { return value == target; }
};


TEST(Map, Basic)
{
    using uavcan::Map;

    static const int POOL_BLOCKS = 3;
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * POOL_BLOCKS, uavcan::MemPoolBlockSize> pool;

    typedef Map<std::string, std::string> MapType;
    std::unique_ptr<MapType> map(new MapType(pool));

    // Empty
    ASSERT_FALSE(map->access("hi"));
    map->remove("foo");
    ASSERT_EQ(0, pool.getNumUsedBlocks());
    ASSERT_FALSE(map->getByIndex(0));
    ASSERT_FALSE(map->getByIndex(1));
    ASSERT_FALSE(map->getByIndex(10000));

    // Insertion
    ASSERT_EQ("a", *map->insert("1", "a"));
    ASSERT_EQ("b", *map->insert("2", "b"));
    ASSERT_EQ(1, pool.getNumUsedBlocks());
    ASSERT_EQ(2, map->getSize());

    // Ordering
    ASSERT_TRUE(map->getByIndex(0)->match("1"));
    ASSERT_TRUE(map->getByIndex(1)->match("2"));

    // Insertion
    ASSERT_EQ("c", *map->insert("3", "c"));
    ASSERT_EQ(1, pool.getNumUsedBlocks());

    ASSERT_EQ("d", *map->insert("4", "d"));
    ASSERT_EQ(2, pool.getNumUsedBlocks());       // Assuming that at least 2 items fit one block
    ASSERT_EQ(4, map->getSize());

    // Making sure everything is here
    ASSERT_EQ("a", *map->access("1"));
    ASSERT_EQ("b", *map->access("2"));
    ASSERT_EQ("c", *map->access("3"));
    ASSERT_EQ("d", *map->access("4"));
    ASSERT_FALSE(map->access("hi"));

    // Modifying existing entries
    *map->access("1") = "A";
    *map->access("2") = "B";
    *map->access("3") = "C";
    *map->access("4") = "D";
    ASSERT_EQ("A", *map->access("1"));
    ASSERT_EQ("B", *map->access("2"));
    ASSERT_EQ("C", *map->access("3"));
    ASSERT_EQ("D", *map->access("4"));

    // Finding some keys
    ASSERT_EQ("1", *map->find(KeyFindPredicate("1")));
    ASSERT_EQ("2", *map->find(KeyFindPredicate("2")));
    ASSERT_EQ("3", *map->find(KeyFindPredicate("3")));
    ASSERT_EQ("4", *map->find(KeyFindPredicate("4")));
    ASSERT_FALSE(map->find(KeyFindPredicate("nonexistent_key")));

    // Finding some values
    ASSERT_EQ("1", *map->find(ValueFindPredicate("A")));
    ASSERT_EQ("2", *map->find(ValueFindPredicate("B")));
    ASSERT_EQ("3", *map->find(ValueFindPredicate("C")));
    ASSERT_EQ("4", *map->find(ValueFindPredicate("D")));
    ASSERT_FALSE(map->find(KeyFindPredicate("nonexistent_value")));

    // Removing one
    map->remove("1");                             // One of dynamics now migrates to the static storage
    map->remove("foo");                           // There's no such thing anyway
    ASSERT_EQ(2, pool.getNumUsedBlocks());
    ASSERT_EQ(3, map->getSize());

    ASSERT_FALSE(map->access("1"));
    ASSERT_EQ("B", *map->access("2"));
    ASSERT_EQ("C", *map->access("3"));
    ASSERT_EQ("D", *map->access("4"));

    // Removing another
    map->remove("2");
    ASSERT_EQ(2, map->getSize());
    ASSERT_EQ(2, pool.getNumUsedBlocks());

    ASSERT_FALSE(map->access("1"));
    ASSERT_FALSE(map->access("2"));
    ASSERT_EQ("C", *map->access("3"));
    ASSERT_EQ("D", *map->access("4"));

    // Adding some new
    unsigned max_key_integer = 0;
    for (int i = 0; i < 100; i++)
    {
        const std::string key   = toString(i);
        const std::string value = toString(i);
        std::string* res = map->insert(key, value);  // Will override some from the above
        if (res == UAVCAN_NULLPTR)
        {
            ASSERT_LT(2, i);
            break;
        }
        else
        {
            ASSERT_EQ(value, *res);
        }
        max_key_integer = unsigned(i);
    }
    std::cout << "Max key/value: " << max_key_integer << std::endl;
    ASSERT_LT(4, max_key_integer);

    // Making sure there is true OOM
    ASSERT_EQ(0, pool.getNumFreeBlocks());
    ASSERT_FALSE(map->insert("nonexistent", "value"));
    ASSERT_FALSE(map->access("nonexistent"));
    ASSERT_FALSE(map->access("value"));

    // Removing odd values - nearly half of them
    map->removeAllWhere(oddValuePredicate);

    // Making sure there's no odd values left
    for (unsigned kv_int = 0; kv_int <= max_key_integer; kv_int++)
    {
        const std::string* val = map->access(toString(kv_int));
        if (val)
        {
            ASSERT_FALSE(kv_int & 1);
        }
        else
        {
            ASSERT_TRUE(kv_int & 1);
        }
    }

    // Making sure the memory will be released
    map.reset();
    ASSERT_EQ(0, pool.getNumUsedBlocks());
}
#endif


TEST(Map, PrimitiveKey)
{
    using uavcan::Map;

    static const int POOL_BLOCKS = 3;
    uavcan::PoolAllocator<uavcan::MemPoolBlockSize * POOL_BLOCKS, uavcan::MemPoolBlockSize> pool;

    typedef Map<short, short> MapType;
    std::unique_ptr<MapType> map(new MapType(pool));

    // Empty
    ASSERT_FALSE(map->access(1));
    map->remove(8);
    ASSERT_EQ(0, pool.getNumUsedBlocks());
    ASSERT_EQ(0, map->getSize());
    ASSERT_FALSE(map->getByIndex(0));

    // Insertion
    ASSERT_EQ(1, *map->insert(1, 1));
    ASSERT_EQ(1, map->getSize());
    ASSERT_EQ(2, *map->insert(2, 2));
    ASSERT_EQ(2, map->getSize());
    ASSERT_EQ(3, *map->insert(3, 3));
    ASSERT_EQ(4, *map->insert(4, 4));
    ASSERT_EQ(4, map->getSize());

    // Ordering
    ASSERT_TRUE(map->getByIndex(0)->match(1));
    ASSERT_TRUE(map->getByIndex(1)->match(2));
    ASSERT_TRUE(map->getByIndex(2)->match(3));
    ASSERT_TRUE(map->getByIndex(3)->match(4));
    ASSERT_FALSE(map->getByIndex(5));
    ASSERT_FALSE(map->getByIndex(1000));
}
