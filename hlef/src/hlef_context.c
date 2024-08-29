#include "hlef_context.h"
#include "hlef_scan.h"

#include <stdio.h>
#include <string.h>

#include "hlef_patch.h"
#include "hlef_interfaces.h"

struct hlef_context hlef_context = {/* ZERO INITIALIZED */};

#define HLEF_NAME_ENTRY(name) [k_hlef_entry_##name] = #name
const char* const hlef_entry_point_names[k_hlef_entry_points] = {
    HLEF_NAME_ENTRY(collision_bsp_test_vector)
};
#undef HLEF_NAME_ENTRY

#ifdef __GNUC__
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#endif // __GNUC__
static const struct hlef_scan_signature collision_bsp_test_vector_signature = {
    (void**)&HLEF_ENTRY_POINT(collision_bsp_test_vector),
    NULL,
    "6a 00 6a 00 56 e8 ?? ?? ?? ?? 83 c4 20",
    // NOTE: missing field initializer warnings are OK here
    HLEF_SCAN_STEPS(
        {k_hlef_scan_translate, 6},
        {k_hlef_scan_read_rel32, 4}, // save the function ptr, for debugging
        // Place a jump to our replacement at the beginning of this function
        {k_hlef_scan_rel32, 4}, // Move cursor to entry point body
        {k_hlef_scan_write_memory,
            .src = "\xE9"}, // JMP rel32 opcode
        {k_hlef_scan_translate, 1},
        {k_hlef_scan_write_rel32, 4,
            .dst=&hlef_exotic_collision_bsp_test_vector} // operand for JMP rel32
    )
};
#ifdef __GNUC__
# pragma GCC diagnostic pop
#endif // __GNUC__
    
static
const struct hlef_scan_signature* const hlef_scan_signatures[] = {
    // ENTRY POINTS
    &collision_bsp_test_vector_signature,
   
    // SENTINEL
    NULL
};

static
int hlef_check_context(const struct hlef_context *ctx)
{    
    if (!ctx)
        return 1;
    
    if (ctx->size_bytes != sizeof(*ctx))
        return 1;
    
    {
        const hlef_entry_point *entry = ctx->entry_points;
        const hlef_entry_point *entry_points_end = entry + k_hlef_entry_points;
        for (; entry != entry_points_end; ++entry) {
            if (!*entry)
                return 1;
        }
    }
    
    return 0;
}

static
void hlef_dump_context(FILE *stream, const struct hlef_context *ctx)
{
    fprintf(stream, "hlef: dumping context (%p)\n", (void*)ctx);
    fprintf(stream, "size_bytes = %zu\n", ctx->size_bytes);
    
    printf("entry points: \n");
    for (int i = 0; i < k_hlef_entry_points; ++i) {
        fprintf(
            stream,
            "\t%-28s 0x%08lX\n", 
            hlef_entry_point_names[i],
            (unsigned long)ctx->entry_points[i]);
    }
}

int hlef_load()
{
    hlef_patches_init();
    
    memset(&hlef_context, 0, sizeof(hlef_context));
    hlef_context.size_bytes = sizeof(hlef_context);
    
    printf("hlef: scanning\n");
    const struct hlef_scan_signature* const* signature = hlef_scan_signatures;
    for (; *signature; ++signature) {
        hlef_scan(*signature);
    }
    
    const int error = hlef_check_context(&hlef_context);
#ifdef HLEF_DUMP_CONTEXT
    hlef_dump_context(stdout, &hlef_context);
#else
    if (error) {
        hlef_dump_context(stdout, &hlef_context);
    }
#endif // HLEF_DUMP_CONTEXT
    
    if (error) {
        hlef_patches_destroy();
    }
    
    return error;
}

void hlef_unload()
{
    hlef_patches_destroy();
}