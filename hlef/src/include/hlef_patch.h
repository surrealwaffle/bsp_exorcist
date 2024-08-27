#ifndef HLEF_PATCH_H
#define HLEF_PATCH_H

#include "stddef.h"

struct hlef_patch
{
    void*  address;
    size_t restore_data_offset;
    size_t restore_data_len;
};

void hlef_patches_init();

void hlef_patches_add(
    const void* data,
    size_t      len);

void hlef_patches_destroy();

#endif // HLEF_PATCH_H