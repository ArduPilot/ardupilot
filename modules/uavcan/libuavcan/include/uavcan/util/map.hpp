/*
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_UTIL_MAP_HPP_INCLUDED
#define UAVCAN_UTIL_MAP_HPP_INCLUDED

#include <cassert>
#include <cstdlib>
#include <uavcan/util/linked_list.hpp>
#include <uavcan/build_config.hpp>
#include <uavcan/dynamic_memory.hpp>
#include <uavcan/util/templates.hpp>
#include <uavcan/util/placement_new.hpp>

namespace uavcan
{
/**
 * Slow but memory efficient KV container.
 *
 * KV pairs will be allocated in the node's memory pool.
 *
 * Please be aware that this container does not perform any speed optimizations to minimize memory footprint,
 * so the complexity of most operations is O(N).
 *
 * Type requirements:
 *  Both key and value must be copyable, assignable and default constructible.
 *  Key must implement a comparison operator.
 *  Key's default constructor must initialize the object into invalid state.
 *  Size of Key + Value + padding must not exceed MemPoolBlockSize.
 */
template <typename Key, typename Value>
class UAVCAN_EXPORT Map : Noncopyable
{
public:
    struct KVPair
    {
        Value value;    // Key and value are swapped because this may allow to reduce padding (depending on types)
        Key key;

        KVPair() :
            value(),
            key()
        { }

        KVPair(const Key& arg_key, const Value& arg_value) :
            value(arg_value),
            key(arg_key)
        { }

        bool match(const Key& rhs) const { return rhs == key; }
    };

private:
    struct KVGroup : LinkedListNode<KVGroup>
    {
        enum { NumKV = (MemPoolBlockSize - sizeof(LinkedListNode<KVGroup>)) / sizeof(KVPair) };
        KVPair kvs[NumKV];

        KVGroup()
        {
            StaticAssert<(static_cast<unsigned>(NumKV) > 0)>::check();
            IsDynamicallyAllocatable<KVGroup>::check();
        }

        static KVGroup* instantiate(IPoolAllocator& allocator)
        {
            void* const praw = allocator.allocate(sizeof(KVGroup));
            if (praw == UAVCAN_NULLPTR)
            {
                return UAVCAN_NULLPTR;
            }
            return new (praw) KVGroup();
        }

        static void destroy(KVGroup*& obj, IPoolAllocator& allocator)
        {
            if (obj != UAVCAN_NULLPTR)
            {
                obj->~KVGroup();
                allocator.deallocate(obj);
                obj = UAVCAN_NULLPTR;
            }
        }

        KVPair* find(const Key& key)
        {
            for (unsigned i = 0; i < static_cast<unsigned>(NumKV); i++)
            {
                if (kvs[i].match(key))
                {
                    return kvs + i;
                }
            }
            return UAVCAN_NULLPTR;
        }
    };

    LinkedListRoot<KVGroup> list_;
    IPoolAllocator& allocator_;

    KVPair* findKey(const Key& key);

    void compact();

    struct YesPredicate
    {
        bool operator()(const Key&, const Value&) const { return true; }
    };

public:
    Map(IPoolAllocator& allocator) :
        allocator_(allocator)
    {
        UAVCAN_ASSERT(Key() == Key());
    }

    ~Map()
    {
        clear();
    }

    /**
     * Returns null pointer if there's no such entry.
     */
    Value* access(const Key& key);

    /**
     * If entry with the same key already exists, it will be replaced
     */
    Value* insert(const Key& key, const Value& value);

    /**
     * Does nothing if there's no such entry.
     */
    void remove(const Key& key);

    /**
     * Removes entries where the predicate returns true.
     * Predicate prototype:
     *  bool (Key& key, Value& value)
     */
    template <typename Predicate>
    void removeAllWhere(Predicate predicate);

    /**
     * Returns first entry where the predicate returns true.
     * Predicate prototype:
     *  bool (const Key& key, const Value& value)
     */
    template <typename Predicate>
    const Key* find(Predicate predicate) const;

    /**
     * Removes all items.
     */
    void clear();

    /**
     * Returns a key-value pair located at the specified position from the beginning.
     * Note that any insertion or deletion may greatly disturb internal ordering, so use with care.
     * If index is greater than or equal the number of pairs, null pointer will be returned.
     */
    KVPair* getByIndex(unsigned index);
    const KVPair* getByIndex(unsigned index) const;

    /**
     * Complexity is O(1).
     */
    bool isEmpty() const { return find(YesPredicate()) == UAVCAN_NULLPTR; }

    /**
     * Complexity is O(N).
     */
    unsigned getSize() const;
};

// ----------------------------------------------------------------------------

/*
 * Map<>
 */
template <typename Key, typename Value>
typename Map<Key, Value>::KVPair* Map<Key, Value>::findKey(const Key& key)
{
    KVGroup* p = list_.get();
    while (p)
    {
        KVPair* const kv = p->find(key);
        if (kv)
        {
            return kv;
        }
        p = p->getNextListNode();
    }
    return UAVCAN_NULLPTR;
}

template <typename Key, typename Value>
void Map<Key, Value>::compact()
{
    KVGroup* p = list_.get();
    while (p)
    {
        KVGroup* const next = p->getNextListNode();
        bool remove_this = true;
        for (int i = 0; i < KVGroup::NumKV; i++)
        {
            if (!p->kvs[i].match(Key()))
            {
                remove_this = false;
                break;
            }
        }
        if (remove_this)
        {
            list_.remove(p);
            KVGroup::destroy(p, allocator_);
        }
        p = next;
    }
}

template <typename Key, typename Value>
Value* Map<Key, Value>::access(const Key& key)
{
    UAVCAN_ASSERT(!(key == Key()));
    KVPair* const kv = findKey(key);
    return kv ? &kv->value : UAVCAN_NULLPTR;
}

template <typename Key, typename Value>
Value* Map<Key, Value>::insert(const Key& key, const Value& value)
{
    UAVCAN_ASSERT(!(key == Key()));
    remove(key);

    KVPair* const kv = findKey(Key());
    if (kv)
    {
        *kv = KVPair(key, value);
        return &kv->value;
    }

    KVGroup* const kvg = KVGroup::instantiate(allocator_);
    if (kvg == UAVCAN_NULLPTR)
    {
        return UAVCAN_NULLPTR;
    }
    list_.insert(kvg);
    kvg->kvs[0] = KVPair(key, value);
    return &kvg->kvs[0].value;
}

template <typename Key, typename Value>
void Map<Key, Value>::remove(const Key& key)
{
    UAVCAN_ASSERT(!(key == Key()));
    KVPair* const kv = findKey(key);
    if (kv)
    {
        *kv = KVPair();
        compact();
    }
}

template <typename Key, typename Value>
template <typename Predicate>
void Map<Key, Value>::removeAllWhere(Predicate predicate)
{
    unsigned num_removed = 0;

    KVGroup* p = list_.get();
    while (p != UAVCAN_NULLPTR)
    {
        KVGroup* const next_group = p->getNextListNode();

        for (int i = 0; i < KVGroup::NumKV; i++)
        {
            const KVPair* const kv = p->kvs + i;
            if (!kv->match(Key()))
            {
                if (predicate(kv->key, kv->value))
                {
                    num_removed++;
                    p->kvs[i] = KVPair();
                }
            }
        }

        p = next_group;
    }

    if (num_removed > 0)
    {
        compact();
    }
}

template <typename Key, typename Value>
template <typename Predicate>
const Key* Map<Key, Value>::find(Predicate predicate) const
{
    KVGroup* p = list_.get();
    while (p != UAVCAN_NULLPTR)
    {
        KVGroup* const next_group = p->getNextListNode();

        for (int i = 0; i < KVGroup::NumKV; i++)
        {
            const KVPair* const kv = p->kvs + i;
            if (!kv->match(Key()))
            {
                if (predicate(kv->key, kv->value))
                {
                    return &p->kvs[i].key;
                }
            }
        }

        p = next_group;
    }
    return UAVCAN_NULLPTR;
}

template <typename Key, typename Value>
void Map<Key, Value>::clear()
{
    removeAllWhere(YesPredicate());
}

template <typename Key, typename Value>
typename Map<Key, Value>::KVPair* Map<Key, Value>::getByIndex(unsigned index)
{
    // Slowly crawling through the dynamic storage
    KVGroup* p = list_.get();
    while (p != UAVCAN_NULLPTR)
    {
        KVGroup* const next_group = p->getNextListNode();

        for (int i = 0; i < KVGroup::NumKV; i++)
        {
            KVPair* const kv = p->kvs + i;
            if (!kv->match(Key()))
            {
                if (index == 0)
                {
                    return kv;
                }
                index--;
            }
        }

        p = next_group;
    }

    return UAVCAN_NULLPTR;
}

template <typename Key, typename Value>
const typename Map<Key, Value>::KVPair* Map<Key, Value>::getByIndex(unsigned index) const
{
    return const_cast<Map<Key, Value>*>(this)->getByIndex(index);
}

template <typename Key, typename Value>
unsigned Map<Key, Value>::getSize() const
{
    unsigned num = 0;
    KVGroup* p = list_.get();
    while (p)
    {
        for (int i = 0; i < KVGroup::NumKV; i++)
        {
            const KVPair* const kv = p->kvs + i;
            if (!kv->match(Key()))
            {
                num++;
            }
        }
        p = p->getNextListNode();
    }
    return num;
}

}

#endif // UAVCAN_UTIL_MAP_HPP_INCLUDED
