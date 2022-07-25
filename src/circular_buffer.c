/**
    generic circular buffer (mem_allocator based)

*/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "circular_buffer.h"
#include "mem_allocator.h"

/**
    @fn circular_buffer_t* Cbu_create(int elements, size_t element_size)
    @brief Create a new circular buffer
    @param elements The maximum number of elements capable of being stored in
        the buffer
    @param element_size The size (in bytes) of the data elements to store
*/
circular_buffer_t* CBUF_create(int elements, size_t element_size)
{
    /* Result is null on error */
    circular_buffer_t* result = NULL;

    /* Allocate a new circular buffer structure */
    result = MEM_alloc(sizeof(circular_buffer_t));

    /* If malloc was successul, initialise the structure */
    if (result != NULL)
    {
        /* General setup */
        result->p__get = 0;
        result->p__put = 0;
        result->p__elements = elements;
        result->p__element_size = element_size;
        result->data_available = false;

        /* Allocate the memory required by the buffer to store data */
        result->p__data = MEM_alloc(elements * element_size);

        if (result->p__data == NULL)
        {
            /* Cannot allocate the memory required for the circular buffer */
            MEM_free(result);
            result = NULL;
        }
    }

    return result;
}


/**
    @fn bool Cbu_put(circular_buffer_t* cb, void* data_in)
    @brief Put a data element into the buffer
    @param cb A pointer to the circular buffer structure
    @param data_in A pointer to the data to add to the buffer
*/
bool CBUF_put(circular_buffer_t* cb, void* data_in)
{
    bool result = false;

    /* Do not overwrite data in the buffer if the buffer is already full! */
    if ((cb->p__put != cb->p__get) || (!cb->data_available))
    {
        /* There is space in the buffer to put new data */
        /* Copy the target to the data buffer */
        memcpy((void*)(cb->p__data + (cb->p__put * cb->p__element_size)),
               data_in,
               cb->p__element_size);

        /* Update the data put pointer */
        cb->p__put++;

        /* Limit to the size of the buffer */
        if (cb->p__put >= cb->p__elements)
        {
            cb->p__put = 0;
        }

        /* Successfully put a new data element in the buffer */
        result = true;

        /* There is data available */
        cb->data_available = true;
    }

    return result;
}


/**
    @fn bool Cbu_get(circular_buffer_t* cb, void* data_out)
    @brief Get a data element from the circular buffer
    @param cb A pointer to the circular buffer structure
    @param data_out A pointer to a buffer to copy the element to
*/
bool CBUF_get(circular_buffer_t* cb, void* data_out)
{
    bool result = false;

    /* Only retrieve data if there is data in the buffer! */
    if (cb->data_available)
    {
        /* Copy the element from the buffer to the target */
        memcpy(data_out,
               (void*)(cb->p__data + (cb->p__get * cb->p__element_size)),
               cb->p__element_size);

        /* advance the get pointer */
        cb->p__get++;

        /* Limit the get pointer to the size of the buffer */
        if (cb->p__get >= cb->p__elements)
            cb->p__get = 0;

        /* There is no data available if put and get are the same value
           after having retrieved an element */
        if (cb->p__get == cb->p__put)
            cb->data_available = false;

        /* Successfully got a data element */
        result = true;
    }

    return result;
}
