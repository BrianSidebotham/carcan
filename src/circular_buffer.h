/**
*/

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H 1

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/** A circular buffer type - every field is private, no fields should be read
    or written by user code, except for @a data_available which may be read */
typedef struct {
    /** PRIVATE: Where to get the next element from */
    int p__get;

    /** PRIVATE: Where to put the next element */
    int p__put;

    /** PRIVATE: A pointer to the data buffer */
    uint8_t* p__data;

    /** PRIVATE: The number of elements that can be held in the buffer */
    int p__elements;

    /** PRIVATE: The size of one element */
    size_t p__element_size;

    /** true when data is in the buffer */
    bool data_available;

    } circular_buffer_t;

/* Extern prototype definitions */
extern circular_buffer_t* CBUF_create(int elements, size_t element_size);
extern bool CBUF_put(circular_buffer_t* cb, void* data_in);
extern bool CBUF_get(circular_buffer_t* cb, void* data_out);

#endif
