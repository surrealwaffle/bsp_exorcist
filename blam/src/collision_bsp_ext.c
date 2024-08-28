#include "blam/collision_bsp_ext.h"

#include <stdbool.h>

#include <assert.h>

#include "blam/collision.h"

// ---------------------------------------------------------------------------------
// INTERNAL DECLARATIONS, STRUCTURES, ENUMS

struct test_vector_context
{
  const struct blam_collision_bsp *bsp; ///< The collision BSP to test against.
  
  const blam_real3d *origin; ///< The tested vector origin.
  const blam_real3d *delta;  ///< The tested vector endpoint, relative to #origin.
  blam_index_long    plane_ignore; ///< This plane is dropped from tests.
  
  // ---------------------------------
  // Immediate History Values
  blam_index_long         leaf;      ///< The index of the previous leaf visited.
  enum blam_bsp_leaf_type leaf_type; ///< The category of the previous leaf visited.
  blam_index_long plane;             ///< The index of the last plane crossed.
};

static
int test_vector_node(
  struct test_vector_context *ctx,
  blam_index_long             root,
  blam_real                   fraction);

static
int test_vector_leaf(
  struct test_vector_context *ctx,
  blam_index_long             leaf,
  blam_real                   fraction);

/**
 * \brief Checks if a BSP leaf contains a BSP2D reference on a given plane.
 *
 * \param [in] bsp         The collision BSP.
 * \param [in] leaf_index  The index of the leaf to check.
 * \param [in] plane_index The index of the plane to check for.
 *
 * \return `true` if the plane is referenced in the leaf, otherwise `false`.
 */
static
bool search_leaf(
  const struct blam_collision_bsp *bsp,
  blam_index_long                  leaf_index,
  blam_index_long                  plane_index);

// ---------------------------------------------------------------------------------
// EXPOSED API

int blamext_collision_bsp_test_vector_next_surface_orientation(
  const struct blam_collision_bsp *bsp,
  const blam_real3d               *origin,
  const blam_real3d               *delta,
  blam_real                        fraction,
  blam_index_long                  last_plane)
{
  assert(bsp);
  assert(origin);
  assert(delta);
  
  struct test_vector_context ctx = 
  {
    .bsp          = bsp,
    .origin       = origin,
    .delta        = delta,
    .plane_ignore = last_plane,
    
    .leaf      = -1,
    .leaf_type = k_bsp_leaf_type_none,
    .plane     = -1
  };
  
  const blam_index_long root = 0;
  
  return test_vector_node(&ctx, root, fraction);
}

// ---------------------------------------------------------------------------------
// INTERNAL FUNCTIONS

int test_vector_node(
  struct test_vector_context *ctx,
  blam_index_long             root,
  blam_real                   fraction)
{
  if (root < 0)
  {
    // the node is a leaf, so test it as a leaf
    const blam_index_long leaf = blam_sanitize_long_s(root);
    return test_vector_leaf(ctx, leaf, fraction);
  }
  
  const struct blam_bsp3d_node *const node  = BLAM_TAG_BLOCK_GET(ctx->bsp, node, bsp3d_nodes, root);
  const struct blam_plane3d *const    plane = BLAM_TAG_BLOCK_GET(ctx->bsp, plane, planes, node->plane);
  
  const blam_real test_origin = blam_plane3d_test(plane, ctx->origin);
  const blam_real dot_delta   = blam_real3d_dot(&plane->normal, ctx->delta);
  const blam_real point_test  = test_origin + fraction * dot_delta;
  const bool any_before = (point_test < 0.0f)  || !(dot_delta >= 0.0f);
  const bool any_after  = (point_test >= 0.0f) || !!(dot_delta >= 0.0f);
  
  if (!any_before || !any_after)
  {
    const blam_index_long new_root = node->children[any_after ? 1 : 0];
    return test_vector_node(ctx, new_root, fraction);
  } else
  {
    // true iff the plane normal faces the vector direction
    const bool plane_faces_forward = !(dot_delta >= 0.0f);
    const blam_index_long first_child  = node->children[plane_faces_forward ? 1 : 0];
    const blam_index_long second_child = node->children[plane_faces_forward ? 0 : 1];
    const blam_real intersection = -(test_origin / dot_delta);
    
    // Where's the elvis operator when you need him?
    // return test_vector_node(ctx, first_child, fraction)
    //   ?: test_vector_node(ctx, second_child, intersection);
    int result = test_vector_node(ctx, first_child, fraction);
    if (result)
      return result;
    
    ctx->plane = node->plane;
    result = test_vector_node(ctx, second_child, intersection);
    return result;
  }
}

int test_vector_leaf(
  struct test_vector_context *ctx,
  blam_index_long             leaf,
  blam_real                   fraction)
{
  enum
  {
    k_surface_direction_front_facing = -1,
    k_surface_direction_no_surface   = 0,
    k_surface_direction_back_facing  = 1,
  };
  
  const enum blam_bsp_leaf_type leaf_type = blam_collision_bsp_classify_leaf(ctx->bsp, leaf);
  BLAM_ASSUME(k_bsp_leaf_type_none <= leaf_type && leaf_type <= k_bsp_leaf_type_exterior);
  BLAM_ASSUME(k_bsp_leaf_type_none <= ctx->leaf_type && ctx->leaf_type <= k_bsp_leaf_type_exterior);
  
  blam_index_long tested_leaf = -1;
  int surface_direction = 0; // no surface
  bool splits_interior = false;
  
  if (blam_bsp_leaf_type_interior(ctx->leaf_type) && leaf_type == k_bsp_leaf_type_exterior)
  {
    // Testing front-facing surfaces
    // Plane splits BSP interior at ctx->leaf from BSP exterior at leaf.
    tested_leaf       = ctx->leaf;
    splits_interior   = false;
    surface_direction = k_surface_direction_front_facing;
  } else if (ctx->leaf_type == k_bsp_leaf_type_exterior && blam_bsp_leaf_type_interior(leaf_type))
  {
    // Testing back-facing surfaces
    // Plane splits BSP exterior at ctx->leaf from BSP interior at leaf.
    tested_leaf       = leaf;
    splits_interior   = false;
    surface_direction = k_surface_direction_back_facing;
  } else if (ctx->leaf_type == k_bsp_leaf_type_double_sided && leaf_type == k_bsp_leaf_type_double_sided)
  {
    // Testing double-sided surfaces
    // Plane splits BSP interior leaves at ctx->leaf and leaf.
    // CONTINUE; TEST ONLY AT PROPER INTERIOR-EXTERIOR SPLITS
    // This keeps the code in search_leaf very sane.
    /*
    tested_leaf       = ctx->leaf;
    splits_interior   = false;
    surface_direction = k_surface_direction_front_facing;
    */
  } else
  {
    // CONTINUE; NO LEAF TO TEST
  }
  
  if (tested_leaf != -1 && ctx->plane_ignore != ctx->plane)
  {
    if (search_leaf(ctx->bsp, tested_leaf, ctx->plane))
      return surface_direction;
    else
      ; // CONTINUE; NO SURFACE INTERSECTED IN THIS LEAF
  }
  
  ctx->leaf      = leaf;
  ctx->leaf_type = leaf_type;
  
  return k_surface_direction_no_surface;
}

bool search_leaf(
  const struct blam_collision_bsp *bsp,
  blam_index_long                  leaf_index,
  blam_index_long                  plane_index)
{
  assert(bsp);
  typedef struct blam_bsp3d_leaf      leaf_type;
  typedef struct blam_bsp2d_reference reference_type;
  
  const leaf_type *const      leaf       = BLAM_TAG_BLOCK_GET(bsp, leaf, leaves, leaf_index);
  const reference_type* const references = BLAM_TAG_BLOCK_BASE(&bsp->bsp2d, references, references) + leaf->first_reference;
  
  for (const reference_type *ref = references, *end = references + leaf->reference_count
    ; ref < end
    ; ++ref)
  {
    const blam_index_long reference_plane = blam_sanitize_long(ref->plane);
    if (plane_index == reference_plane)
      return true; // some surface, ANY surface, must have been hit
    
    // NOTE: This function implements no mitigations to phantom BSP nor BSP holes.
  }
  
  return false;
}