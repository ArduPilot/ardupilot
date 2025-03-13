/// Source: https://github.com/pavel-kirienko/cavl
///
/// Cavl is a single-header C library providing an implementation of AVL tree suitable for deeply embedded systems.
/// To integrate it into your project, simply copy this file into your source tree. Read the API docs below.
///
/// See also O1Heap <https://github.com/pavel-kirienko/o1heap> -- a deterministic memory manager for hard-real-time
/// high-integrity embedded systems.
///
/// Copyright (c) 2021 Pavel Kirienko <pavel@opencyphal.org>
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
/// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
/// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
/// and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of
/// the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
/// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
/// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
/// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#pragma once

#include "canard.h"

/// Modified for use with Libcanard: use the same assertion check macro if provided.
#ifdef CANARD_ASSERT
#    define CAVL_ASSERT CANARD_ASSERT
#else
// Intentional violation of MISRA: inclusion not at the top of the file to eliminate unnecessary dependency on assert.h.
#    include <assert.h>  // NOSONAR
#    define CAVL_ASSERT assert
#endif

#ifdef __cplusplus
// This is, strictly speaking, useless because we do not define any functions with external linkage here,
// but it tells static analyzers that what follows should be interpreted as C code rather than C++.
extern "C" {
#endif

// ----------------------------------------         PUBLIC API SECTION         ----------------------------------------

/// Modified for use with Libcanard: expose the Cavl structure via public API as CanardTreeNode.
typedef CanardTreeNode Cavl;

/// Returns POSITIVE if the search target is GREATER than the provided node, negative if smaller, zero on match (found).
/// Values other than {-1, 0, +1} are not recommended to avoid overflow during the narrowing conversion of the result.
typedef int8_t (*CavlPredicate)(void* user_reference, const Cavl* node);

/// If provided, the factory will be invoked when the sought node does not exist in the tree.
/// It is expected to return a new node that will be inserted immediately (without the need to traverse the tree again).
/// If the factory returns NULL or is not provided, the tree is not modified.
typedef Cavl* (*CavlFactory)(void* user_reference);

/// Look for a node in the tree using the specified search predicate. The worst-case complexity is O(log n).
/// - If the node is found, return it.
/// - If the node is not found and the factory is NULL, return NULL.
/// - Otherwise, construct a new node using the factory; if the result is not NULL, insert it; return the result.
/// The user_reference is passed into the predicate & factory unmodified.
/// The root node may be replaced in the process.
/// If predicate is NULL, returns NULL.
static inline Cavl* cavlSearch(Cavl** const        root,
                               void* const         user_reference,
                               const CavlPredicate predicate,
                               const CavlFactory   factory);

/// Remove the specified node from its tree. The root node may be replaced in the process.
/// The worst-case complexity is O(log n).
/// The function has no effect if either of the pointers are NULL.
/// If the node is not in the tree, the behavior is undefined; it may create cycles in the tree which is deadly.
/// It is safe to pass the result of cavlSearch() directly as the second argument:
///     cavlRemove(&root, cavlSearch(&root, user_reference, search_predicate, NULL));
/// It is recommended to invalidate the pointers stored in the node after its removal.
static inline void cavlRemove(Cavl** const root, const Cavl* const node);

/// Return the min-/max-valued node stored in the tree, depending on the flag. This is an extremely fast query.
/// Returns NULL iff the argument is NULL (i.e., the tree is empty). The worst-case complexity is O(log n).
static inline Cavl* cavlFindExtremum(Cavl* const root, const bool maximum)
{
    Cavl* result = NULL;
    Cavl* c      = root;
    while (c != NULL)
    {
        result = c;
        c      = c->lr[maximum];
    }
    return result;
}

// ----------------------------------------     END OF PUBLIC API SECTION      ----------------------------------------
// ----------------------------------------      POLICE LINE DO NOT CROSS      ----------------------------------------

/// INTERNAL USE ONLY. Makes the '!r' child of node 'x' its parent; i.e., rotates 'x' toward 'r'.
static inline void cavlPrivateRotate(Cavl* const x, const bool r)
{
    CAVL_ASSERT((x != NULL) && (x->lr[!r] != NULL) && ((x->bf >= -1) && (x->bf <= +1)));
    Cavl* const z = x->lr[!r];
    if (x->up != NULL)
    {
        x->up->lr[x->up->lr[1] == x] = z;
    }
    z->up     = x->up;
    x->up     = z;
    x->lr[!r] = z->lr[r];
    if (x->lr[!r] != NULL)
    {
        x->lr[!r]->up = x;
    }
    z->lr[r] = x;
}

/// INTERNAL USE ONLY.
/// Accepts a node and how its balance factor needs to be changed -- either +1 or -1.
/// Returns the new node to replace the old one if tree rotation took place, same node otherwise.
static inline Cavl* cavlPrivateAdjustBalance(Cavl* const x, const bool increment)
{
    CAVL_ASSERT((x != NULL) && ((x->bf >= -1) && (x->bf <= +1)));
    Cavl*        out    = x;
    const int8_t new_bf = (int8_t) (x->bf + (increment ? +1 : -1));
    if ((new_bf < -1) || (new_bf > 1))
    {
        const bool   r    = new_bf < 0;   // bf<0 if left-heavy --> right rotation is needed.
        const int8_t sign = r ? +1 : -1;  // Positive if we are rotating right.
        Cavl* const  z    = x->lr[!r];
        CAVL_ASSERT(z != NULL);   // Heavy side cannot be empty.
        if ((z->bf * sign) <= 0)  // Parent and child are heavy on the same side or the child is balanced.
        {
            out = z;
            cavlPrivateRotate(x, r);
            if (0 == z->bf)
            {
                x->bf = (int8_t) (-sign);
                z->bf = (int8_t) (+sign);
            }
            else
            {
                x->bf = 0;
                z->bf = 0;
            }
        }
        else  // Otherwise, the child needs to be rotated in the opposite direction first.
        {
            Cavl* const y = z->lr[r];
            CAVL_ASSERT(y != NULL);  // Heavy side cannot be empty.
            out = y;
            cavlPrivateRotate(z, !r);
            cavlPrivateRotate(x, r);
            if ((y->bf * sign) < 0)
            {
                x->bf = (int8_t) (+sign);
                y->bf = 0;
                z->bf = 0;
            }
            else if ((y->bf * sign) > 0)
            {
                x->bf = 0;
                y->bf = 0;
                z->bf = (int8_t) (-sign);
            }
            else
            {
                x->bf = 0;
                z->bf = 0;
            }
        }
    }
    else
    {
        x->bf = new_bf;  // Balancing not needed, just update the balance factor and call it a day.
    }
    return out;
}

/// INTERNAL USE ONLY.
/// Takes the culprit node (the one that is added); returns NULL or the root of the tree (possibly new one).
/// When adding a new node, set its balance factor to zero and call this function to propagate the changes upward.
static inline Cavl* cavlPrivateRetraceOnGrowth(Cavl* const added)
{
    CAVL_ASSERT((added != NULL) && (0 == added->bf));
    Cavl* c = added;      // Child
    Cavl* p = added->up;  // Parent
    while (p != NULL)
    {
        const bool r = p->lr[1] == c;  // c is the right child of parent
        CAVL_ASSERT(p->lr[r] == c);
        c = cavlPrivateAdjustBalance(p, r);
        p = c->up;
        if (0 == c->bf)
        {           // The height change of the subtree made this parent perfectly balanced (as all things should be),
            break;  // hence, the height of the outer subtree is unchanged, so upper balance factors are unchanged.
        }
    }
    CAVL_ASSERT(c != NULL);
    return (NULL == p) ? c : NULL;  // New root or nothing.
}

static inline Cavl* cavlSearch(Cavl** const        root,
                               void* const         user_reference,
                               const CavlPredicate predicate,
                               const CavlFactory   factory)
{
    Cavl* out = NULL;
    if ((root != NULL) && (predicate != NULL))
    {
        Cavl*  up = *root;
        Cavl** n  = root;
        while (*n != NULL)
        {
            const int8_t cmp = predicate(user_reference, *n);
            if (0 == cmp)
            {
                out = *n;
                break;
            }
            up = *n;
            n  = &(*n)->lr[cmp > 0];
            CAVL_ASSERT((NULL == *n) || ((*n)->up == up));
        }
        if (NULL == out)
        {
            out = (NULL == factory) ? NULL : factory(user_reference);
            if (out != NULL)
            {
                *n             = out;  // Overwrite the pointer to the new node in the parent node.
                out->lr[0]     = NULL;
                out->lr[1]     = NULL;
                out->up        = up;
                out->bf        = 0;
                Cavl* const rt = cavlPrivateRetraceOnGrowth(out);
                if (rt != NULL)
                {
                    *root = rt;
                }
            }
        }
    }
    return out;
}

static inline void cavlRemove(Cavl** const root, const Cavl* const node)
{
    if ((root != NULL) && (node != NULL))
    {
        CAVL_ASSERT(*root != NULL);  // Otherwise, the node would have to be NULL.
        CAVL_ASSERT((node->up != NULL) || (node == *root));
        Cavl* p = NULL;   // The lowest parent node that suffered a shortening of its subtree.
        bool  r = false;  // Which side of the above was shortened.
        // The first step is to update the topology and remember the node where to start the retracing from later.
        // Balancing is not performed yet so we may end up with an unbalanced tree.
        if ((node->lr[0] != NULL) && (node->lr[1] != NULL))
        {
            Cavl* const re = cavlFindExtremum(node->lr[1], false);
            CAVL_ASSERT((re != NULL) && (NULL == re->lr[0]) && (re->up != NULL));
            re->bf        = node->bf;
            re->lr[0]     = node->lr[0];
            re->lr[0]->up = re;
            if (re->up != node)
            {
                p = re->up;  // Retracing starts with the ex-parent of our replacement node.
                CAVL_ASSERT(p->lr[0] == re);
                p->lr[0] = re->lr[1];  // Reducing the height of the left subtree here.
                if (p->lr[0] != NULL)
                {
                    p->lr[0]->up = p;
                }
                re->lr[1]     = node->lr[1];
                re->lr[1]->up = re;
                r             = false;
            }
            else  // In this case, we are reducing the height of the right subtree, so r=1.
            {
                p = re;    // Retracing starts with the replacement node itself as we are deleting its parent.
                r = true;  // The right child of the replacement node remains the same so we don't bother relinking it.
            }
            re->up = node->up;
            if (re->up != NULL)
            {
                re->up->lr[re->up->lr[1] == node] = re;  // Replace link in the parent of node.
            }
            else
            {
                *root = re;
            }
        }
        else  // Either or both of the children are NULL.
        {
            p             = node->up;
            const bool rr = node->lr[1] != NULL;
            if (node->lr[rr] != NULL)
            {
                node->lr[rr]->up = p;
            }
            if (p != NULL)
            {
                r        = p->lr[1] == node;
                p->lr[r] = node->lr[rr];
                if (p->lr[r] != NULL)
                {
                    p->lr[r]->up = p;
                }
            }
            else
            {
                *root = node->lr[rr];
            }
        }
        // Now that the topology is updated, perform the retracing to restore balance. We climb up adjusting the
        // balance factors until we reach the root or a parent whose balance factor becomes plus/minus one, which
        // means that that parent was able to absorb the balance delta; in other words, the height of the outer
        // subtree is unchanged, so upper balance factors shall be kept unchanged.
        if (p != NULL)
        {
            Cavl* c = NULL;
            for (;;)
            {
                c = cavlPrivateAdjustBalance(p, !r);
                p = c->up;
                if ((c->bf != 0) || (NULL == p))  // Reached the root or the height difference is absorbed by c.
                {
                    break;
                }
                r = p->lr[1] == c;
            }
            if (NULL == p)
            {
                CAVL_ASSERT(c != NULL);
                *root = c;
            }
        }
    }
}

#ifdef __cplusplus
}
#endif
