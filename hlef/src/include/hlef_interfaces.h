#ifndef HLEF_INTERFACES_H
#define HLEF_INTERFACES_H

#include <stdint.h>

#include "blam/base.h"
#include "blam/collision.h"

/* WARNING
 * These functions use an exotic ABI, used to hook Halo's functions.
 * Never call these functions raw.
 */

blam_bool hlef_exotic_collision_bsp_test_vector(
  // REGISTER ARGUMENTS
  blam_flags_long flags,     // ARG.EAX, enum blam_collision_test_flags
  uint32_t        dummy_edx, // ARG.EDX, unused, needed for regparm
  struct blam_collision_bsp_test_vector_result *data, // ARG.ECX

  // STACK ARGUMENTS
  struct blam_collision_bsp *bsp,
  struct blam_bit_vector     breakable_surfaces,
  const blam_real3d         *origin,
  const blam_real3d         *delta,
  blam_real                  max_scale)
  __attribute__((cdecl, regparm(3)));

#endif // HLEF_INTERFACES_H