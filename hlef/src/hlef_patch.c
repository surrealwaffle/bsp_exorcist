#include "hlef_patch.h"

#include <stdlib.h>
#include <string.h>
#include <windows.h>

#include "hlef_vector.h"

static struct hlef_vector   patches                 = {};
static char*                patch_restore_data      = NULL;
static size_t               patch_restore_data_len  = 0;

void hlef_patches_init()
{
    patches = hlef_vector_init(sizeof(struct hlef_patch), NULL, NULL);
    patch_restore_data = NULL;
}

void hlef_patches_add(
    const void* data,
    size_t      len)
{
    if (!data || len == 0)
        return;
    
    {
        char* new_restore_data = realloc(
            patch_restore_data, 
            patch_restore_data_len + len);
        
        if (!new_restore_data)
            exit(EXIT_FAILURE);
        patch_restore_data = new_restore_data;
    }
    
    memcpy(patch_restore_data + patch_restore_data_len, data, len);
    struct hlef_patch patch = {
        .address                = (void*)data,
        .restore_data_offset    = patch_restore_data_len,
        .restore_data_len       = len
    };
    patch_restore_data_len += len;
    
    hlef_vector_push_back(&patches, &patch);
}

void hlef_patches_destroy()
{
    HANDLE hProcess = GetCurrentProcess();
    
    HLEF_VECTOR_RFOR_EACH(struct hlef_patch, patch, &patches) {
        DWORD flOldProtect;
        BOOL success = VirtualProtect(
            patch->address, 
            patch->restore_data_len,
            PAGE_EXECUTE_READWRITE,
            &flOldProtect);
        
        if (!success)
            continue; // best effort
        
        memcpy(
            patch->address,
            patch_restore_data + patch->restore_data_offset,
            patch->restore_data_len);
        
        VirtualProtect(
            patch->address, 
            patch->restore_data_len,
            flOldProtect,
            &flOldProtect);
        
        FlushInstructionCache(
            hProcess, 
            patch->address,
            patch->restore_data_len);
    }
    
    hlef_vector_destroy(&patches);
    patch_restore_data_len = 0;
    patch_restore_data = NULL;
}