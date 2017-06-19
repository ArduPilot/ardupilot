#include <rpcmem.h>

#define QFLIGHT_RPC_ALLOCATE(type) (type *)rpcmem_alloc_def(sizeof(type))
