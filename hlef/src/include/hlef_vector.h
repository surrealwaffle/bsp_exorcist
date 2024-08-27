#ifndef HLEF_VECTOR_H
#define HLEF_VECTOR_H

#include <stddef.h>

struct hlef_vector
{
    struct {
        size_t  stride;   // sizeof(T)
        size_t  capacity; // # of T
        char*   start;
    } buffer;
    
    size_t size;
};

struct hlef_vector hlef_vector_init(
    size_t      stride,
    const void* init,
    const void* init_end);

void hlef_vector_destroy(
    struct hlef_vector *v);

void hlef_vector_reserve(
    struct hlef_vector *v,
    size_t              capacity);

void* hlef_vector_subscript(
    struct hlef_vector *v,
    size_t i);

const void* hlef_vector_csubscript(
    const struct hlef_vector *v,
    size_t i);

void hlef_vector_push_back(
    struct hlef_vector *restrict v,
    const void* restrict         e);

void hlef_vector_pop_back(struct hlef_vector *v);

void* hlef_vector_begin(struct hlef_vector *v);
void* hlef_vector_end(struct hlef_vector *v);

const void* hlef_vector_cbegin(const struct hlef_vector *v);
const void* hlef_vector_cend(const struct hlef_vector *v);

#define HLEF_VECTOR_FOR_EACH(T, it, pv)                                     \
for (T *it             = hlef_vector_begin((pv))                            \
    ,  *end_##__LINE__ = hlef_vector_end((pv))                              \
    ;it != end_##__LINE__                                                   \
    ;++it)

#define HLEF_VECTOR_RFOR_EACH(T, it, pv)                                    \
for (T *it             = (T*)hlef_vector_end((pv))                          \
    ,  *end_##__LINE__ = (T*)hlef_vector_begin((pv))                        \
    ;it-- != end_##__LINE__                                                 \
    ;)

#endif // HLEF_VECTOR_H