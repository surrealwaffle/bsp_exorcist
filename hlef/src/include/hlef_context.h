#ifndef HLEF_CONTEXT_H
#define HLEF_CONTEXT_H

#include <stddef.h>
#include <stdint.h>

#include "hlef_base.h"
#include "blam/collision.h"

#define HLEF_ENTRY_POINT(name) hlef_context.entry_points[k_hlef_entry_ ## name]

#define HLEF_ENTRY_CALL(name, ...) ((hlef_ ## name ## _proc)HLEF_ENTRY_POINT(name))(__VA_ARGS__)

typedef void* hlef_exotic_function;
typedef void* hlef_entry_point;

typedef hlef_exotic_function hlef_collision_bsp_test_vector_proc;

enum hlef_entry_points
{
    k_hlef_entry_collision_bsp_test_vector,
    
    k_hlef_entry_points
};

struct hlef_context
{
    size_t           size_bytes;
    hlef_entry_point entry_points[k_hlef_entry_points];
};

extern struct hlef_context hlef_context;
extern const char* const hlef_entry_point_names[k_hlef_entry_points];

/**
 * \brief Loads the engine interfaces.
 *
 * \return 0 on success, otherwise non-zero.
 */
int hlef_load();

/**
 * \brief Releases the resources held for the engine interfaces.
 */
void hlef_unload();

#endif // HLEF_CONTEXT_H