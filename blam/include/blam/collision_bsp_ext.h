#ifndef BLAM_COLLISION_BSP_EXT_H
#define BLAM_COLLISION_BSP_EXT_H

#include "blam/base.h"
#include "blam/collision.h"
#include "blam/math.h"

/**
 * \brief Tests a vector against a BSP for the next surface orientation.
 *
 * \param [in] bsp        The collision BSP to test against.
 * \param [in] origin     The starting point of the vector.
 * \param [in] delta      The vector endpoint, relative to \a origin.
 * \param [in] fraction   The starting point of the test, as a fraction of \a delta.
 * \param [in] last_plane The index of the last plane traversed. All BSP2D 
 *                        references with this plane are dropped from the test.
 *
 * \return `0` if no surface was hit, or
 *         a negative value if a front-facing surface was hit,
 *         a positive value if a back-facing surface was hit.
 */
int blamext_collision_bsp_test_vector_next_surface_orientation(
  const struct blam_collision_bsp *bsp,
  const blam_real3d               *origin,
  const blam_real3d               *delta,
  blam_real                        fraction,
  blam_index_long                  last_plane);

#endif // BLAM_COLLISION_BSP_EXT_H