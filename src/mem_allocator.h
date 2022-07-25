#ifndef MEM_ALLOCATOR_H
#define MEM_ALLOCATOR_H

#include <stddef.h>

extern void* MEM_alloc(size_t size);
extern void MEM_free(void* data);

#endif