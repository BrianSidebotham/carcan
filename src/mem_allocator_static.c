#include <stdint.h>

static size_t index = 0;
static uint8_t data[MEM_ALLOCATOR_SIZE] = {0};

void* MEM_alloc(size_t size)
{
    void* result = &data[index];

    if( ( index + size ) >= MEM_ALLOCATOR_SIZE )
    {
        return NULL;
    }

    /* Advance the index on the required amount */
    index += size;
    return result;
}

void MEM_free(void* data)
{

}
