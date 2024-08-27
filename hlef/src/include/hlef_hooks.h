#ifndef HLEF_HOOKS_H
#define HLEF_HOOKS_H

#include "blam/base.h"
#include "blam/collision.h"

blam_bool hlef_hook_collision_bsp_test_vector(
  struct blam_collision_bsp *bsp,
  struct blam_bit_vector     breakable_surfaces,
  const blam_real3d         *origin,
  const blam_real3d         *delta,
  blam_real                  max_scale,
  blam_flags_long            flags, // enum blam_collision_test_flags
  struct blam_collision_bsp_test_vector_result *data);

#endif // HLEF_HOOKS_H