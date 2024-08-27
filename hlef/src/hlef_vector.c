#include "hlef_vector.h"

#include <stdlib.h>

#include <assert.h>
#include <string.h>

struct hlef_vector hlef_vector_init(
    size_t      stride,
    const void* init,
    const void* init_end)
{
    assert(init_end >= init);
    assert(((char*)init_end - (char*)init) % stride == 0);
    
    const size_t cb = (size_t)((char*)init_end - (char*)init);
    struct hlef_vector result = {};
    result.buffer.stride = stride;
    result.buffer.capacity = cb / stride;
    
    if (init != NULL && init != init_end) {
        result.buffer.start = malloc(cb);
        if (!result.buffer.start)
            exit(EXIT_FAILURE);
        memcpy(result.buffer.start, init, cb);
    }
    
    return result;
}

void hlef_vector_destroy(
    struct hlef_vector *v)
{
    if (!v)
        return;
    
    free(v->buffer.start);
    memset(v, 0, sizeof(*v));
}

void hlef_vector_reserve(
    struct hlef_vector *v,
    size_t              capacity)
{
    assert(v);
    
    if (capacity < v->buffer.capacity)
        return;
    
    const size_t new_cb = capacity * v->buffer.stride;
    void* new_buf = realloc(v->buffer.start, new_cb);
    if (!new_buf)
        exit(EXIT_FAILURE);

    v->buffer.capacity = capacity;
    v->buffer.start    = new_buf;
}

void* hlef_vector_subscript(
    struct hlef_vector *v,
    size_t i)
{
    assert(v);
    assert(i <= v->size);
    return v->buffer.start + v->buffer.stride * i;
}   

const void* hlef_vector_csubscript(
    const struct hlef_vector *v,
    size_t i)
{
    assert(v);
    assert(i <= v->size);
    return v->buffer.start + v->buffer.stride * i;
}

void hlef_vector_push_back(
    struct hlef_vector *restrict v,
    const void* restrict         e)
{
    assert(v);
    
    if (v->size == v->buffer.capacity) {
        const size_t new_capacity = (size_t)(1.5f * v->size) + 4;
        hlef_vector_reserve(v, new_capacity);
    }
    
    memcpy(hlef_vector_subscript(v, v->size), e, v->buffer.stride);
    ++v->size;
}
    
void hlef_vector_pop_back(struct hlef_vector *v)
{
    --v->size;
}

void* hlef_vector_begin(struct hlef_vector *v)
{
    assert(v);
    return hlef_vector_subscript(v, 0);
}

void* hlef_vector_end(struct hlef_vector *v)
{
    assert(v);
    return hlef_vector_subscript(v, v->size);
}

const void* hlef_vector_cbegin(const struct hlef_vector *v)
{
    assert(v);
    return hlef_vector_csubscript(v, 0);
}

const void* hlef_vector_cend(const struct hlef_vector *v)
{
    assert(v);
    return hlef_vector_csubscript(v, v->size);
}