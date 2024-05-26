/*
 * Coverity Scan model.
 *
 * - A model file can't import any header files.
 * - Therefore only some built-in primitives like int, char and void are
 *   available but not wchar_t, NULL etc.
 * - Modeling doesn't need full structs and typedefs. Rudimentary structs
 *   and similar types are sufficient.
 * - An uninitialized local pointer is not an error. It signifies that the
 *   variable could be either NULL or have some data.
 *
 * Coverity Scan doesn't pick up modifications automatically. The model file
 * must be uploaded by an admin in the analysis settings of
 * https://scan.coverity.com/projects/1513
 */

namespace std
{
    typedef unsigned long size_t;
}

namespace uavcan
{

void handleFatalError(const char* msg)
{
    __coverity_panic__();
}

template <std::size_t PoolSize, std::size_t BlockSize>
class PoolAllocator
{
public:
    void* allocate(std::size_t size)
    {
        return __coverity_alloc__(size);
    }

    void deallocate(const void* ptr)
    {
        __coverity_free__(ptr);
    }
};

}
