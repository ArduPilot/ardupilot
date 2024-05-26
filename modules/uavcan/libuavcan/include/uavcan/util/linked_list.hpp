/*
 * Singly-linked list.
 * Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#ifndef UAVCAN_UTIL_LINKED_LIST_HPP_INCLUDED
#define UAVCAN_UTIL_LINKED_LIST_HPP_INCLUDED

#include <cstdlib>
#include <cassert>
#include <uavcan/build_config.hpp>
#include <uavcan/util/templates.hpp>

namespace uavcan
{
/**
 * Classes that are supposed to be linked-listed should derive this.
 */
template <typename T>
class UAVCAN_EXPORT LinkedListNode : Noncopyable
{
    T* next_;

protected:
    LinkedListNode()
        : next_(UAVCAN_NULLPTR)
    { }

    ~LinkedListNode() { }

public:
    T* getNextListNode() const { return next_; }

    void setNextListNode(T* node)
    {
        next_ = node;
    }
};

/**
 * Linked list root.
 */
template <typename T>
class UAVCAN_EXPORT LinkedListRoot : Noncopyable
{
    T* root_;

public:
    LinkedListRoot()
        : root_(UAVCAN_NULLPTR)
    { }

    T* get() const { return root_; }
    bool isEmpty() const { return get() == UAVCAN_NULLPTR; }

    /**
     * Complexity: O(N)
     */
    unsigned getLength() const;

    /**
     * Inserts the node to the beginning of the list.
     * If the node is already present in the list, it will be relocated to the beginning.
     * Complexity: O(N)
     */
    void insert(T* node);

    /**
     * Inserts the node immediately before the node X where predicate(X) returns true.
     * If the node is already present in the list, it can be relocated to a new position.
     * Complexity: O(2N) (calls remove())
     */
    template <typename Predicate>
    void insertBefore(T* node, Predicate predicate);

    /**
     * Removes only the first occurence of the node.
     * Complexity: O(N)
     */
    void remove(const T* node);
};

// ----------------------------------------------------------------------------

/*
 * LinkedListRoot<>
 */
template <typename T>
unsigned LinkedListRoot<T>::getLength() const
{
    T* node = root_;
    unsigned cnt = 0;
    while (node)
    {
        cnt++;
        node = node->getNextListNode();
    }
    return cnt;
}

template <typename T>
void LinkedListRoot<T>::insert(T* node)
{
    if (node == UAVCAN_NULLPTR)
    {
        UAVCAN_ASSERT(0);
        return;
    }
    remove(node);  // Making sure there will be no loops
    node->setNextListNode(root_);
    root_ = node;
}

template <typename T>
template <typename Predicate>
void LinkedListRoot<T>::insertBefore(T* node, Predicate predicate)
{
    if (node == UAVCAN_NULLPTR)
    {
        UAVCAN_ASSERT(0);
        return;
    }

    remove(node);

    if (root_ == UAVCAN_NULLPTR || predicate(root_))
    {
        node->setNextListNode(root_);
        root_ = node;
    }
    else
    {
        T* p = root_;
        while (p->getNextListNode())
        {
            if (predicate(p->getNextListNode()))
            {
                break;
            }
            p = p->getNextListNode();
        }
        node->setNextListNode(p->getNextListNode());
        p->setNextListNode(node);
    }
}

template <typename T>
void LinkedListRoot<T>::remove(const T* node)
{
    if (root_ == UAVCAN_NULLPTR || node == UAVCAN_NULLPTR)
    {
        return;
    }

    if (root_ == node)
    {
        root_ = root_->getNextListNode();
    }
    else
    {
        T* p = root_;
        while (p->getNextListNode())
        {
            if (p->getNextListNode() == node)
            {
                p->setNextListNode(p->getNextListNode()->getNextListNode());
                break;
            }
            p = p->getNextListNode();
        }
    }
}

}

#endif // UAVCAN_UTIL_LINKED_LIST_HPP_INCLUDED
