#include <stdint.h>
#include <string.h>

void* MEM_alloc(size_t size)
{
    return malloc(size);
}

void MEM_free(void* data)
{
    return free(data);
}
