#include "hlef_hooks.h"

#include <assert.h>
#include <stdio.h>
#include <inttypes.h>

#include "blam/collision_bsp.h"

blam_bool hlef_hook_collision_bsp_test_vector(
  struct blam_collision_bsp *bsp,
  struct blam_bit_vector     breakable_surfaces,
  const blam_real3d         *origin,
  const blam_real3d         *delta,
  blam_real                  max_scale,
  blam_flags_long            flags, // enum blam_collision_test_flags
  struct blam_collision_bsp_test_vector_result *data)
{
  return blam_collision_bsp_test_vector(bsp, breakable_surfaces, origin, delta, max_scale, flags, data);
}
