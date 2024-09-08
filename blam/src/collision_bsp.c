#include "blam/collision_bsp.h"

#include <stdbool.h>
#include <assert.h>
#include <math.h>

#include "blam/math.h"
#include "blam/tag.h"

// -----------------------------------------------------------------------------
// INTERNAL DECLARATIONS, STRUCTURES, ENUMS

static bool mitigate_phantom_bsp = true;

typedef struct blam_collision_bsp collision_bsp;
typedef struct blam_bit_vector    bit_vector;
typedef struct blam_collision_bsp_test_vector_result test_vector_result;

/**
 * \brief Manages additional, non-vanilla state for BSP-vector intersection tests.
 *
 * \sa #test_vector_context
 */
struct test_vector_context_ext
{
  bool just_encountered_leak; ///< \c true if the last solid partition was a leak.
  bool has_pending_result;    ///< \c true if there is a #pending result.
  
  struct 
  {
    blam_real       fraction; ///< The intersection fraction with #surface.
    blam_index_long plane;    ///< The index of the intersected partitioning plane.
    blam_index_long surface;  ///< The index of the intersected surface candidate.
  } pending; ///< May hold an intersection result for verification.
             ///< The result is considered verified and accepted only when the 
             ///< solid partition that follows does not feature a leak.
};

/**
 * \brief Manages the state required for a BSP-vector intersection test.
 */
struct test_vector_context
{
  enum blam_collision_test_flags flags; ///< Masked by \c k_collision_test_bsp_bits.
  const collision_bsp *bsp;    ///< The BSP to test against.
  bit_vector           breakable_surfaces; ///< The state of breakable surfaces.
  const blam_real3d   *origin; ///< The tested vector origin.
  const blam_real3d   *delta;  ///< The tested vector endpoint, relative to #origin.

  test_vector_result  *data;   ///< Receives the intersection result.

  // ---------------------------------
  // Immediate History Values
  blam_index_long         leaf;      ///< The index of the previous leaf visited.
  enum blam_bsp_leaf_type leaf_type; ///< The category of the previous leaf visited.
  blam_index_long         plane;     ///< The index of the last plane crossed.
  
  struct test_vector_context_ext ext; ///< (NON-VANILLA) Extended context data.
};

/**
 * \brief Attempts to commit a surface intersection result to the context object.
 *
 * \param [in,out] ctx           The test context.
 * \param [in]     fraction      The proportional intersection distance.
 * \param [in]     plane_index   The index of the intersected plane.
 * \param [in]     surface_index The index of the intersected surface.
 * 
 * \return \c true if the result was committed, otherwise \c false.
 */
static
blam_bool test_vector_context_try_commit_result(
  struct test_vector_context *ctx,
  blam_real       fraction,
  blam_index_long plane_index,
  blam_index_long surface_index);

/**
 * \brief Attempts to commit a pending intersection result, if any.
 *
 * \param [in,out] ctx The test context.
 *
 * \return \c true if there was a pending result in the context data and it was 
 *         committed, otherwise \c false.
 */
static
blam_bool test_vector_context_try_commit_pending_result(
  struct test_vector_context *ctx);

/**
 * \brief Tests a vector against a collision BSP subtree.
 *
 * \param [in,out] ctx      The test context.
 * \param [in]     root     The index of the subtree root node.
 *                          If this value is negative, it is treated as a leaf node.
 * \param [in]     fraction The starting distance from the test origin, as a 
 *                          fraction of `ctx->delta`.
 * \param [in]     terminal The maximum distance from the test origin, as a fraction
 *                          of `ctx->delta`.
 *
 * \return \c true if a surface was intersected, otherwise \c false.
 */
static
blam_bool collision_bsp_test_vector_node(
  struct test_vector_context *ctx,
  blam_index_long root,
  blam_real       fraction, 
  blam_real       terminal
);

/**
 * \brief Tests a vector against a collision BSP subtree.
 *
 * \param [in,out] ctx      The test context.
 * \param [in]     leaf     The index of the leaf node.
 *                          If this value is negative, it is treated as a leaf node.
 * \param [in]     fraction The starting distance from the test origin, as a 
 *                          fraction of `ctx->delta`.
 *
 * \return \c true if a surface was intersected, otherwise \c false.
 */
static
blam_bool collision_bsp_test_vector_leaf(
  struct test_vector_context *const ctx,
  blam_index_long leaf,
  blam_real       fraction);

/**
 * \brief Tests a vector against a BSP leaf for an intersected surface.
 *
 * The point tested is calculated by `fraction * delta + origin`.
 * This point should be on the plane referred to by \a plane_index.
 *
 * \param [in] bsp                The collision BSP.
 * \param [in] breakable_surfaces The state of breakable surfaces.
 * \param [in] leaf_index         The index of the leaf.
 * \param [in] plane_index        The index of the intersected plane.
 * \param [in] splits_interior    If \c true, the intersected plane divides two BSP 
 *                                interior leaves.
 * \param [in] origin             The vector starting point.
 * \param [in] delta              The vector endpoint, relative to \a origin.
 * \param [in] fraction           The distance to the test point, as a fraction of 
 *                                \a delta.
 *
 * \return The index of the intersected surface, or `-1` if no surface was hit.
 */
static
blam_index_long collision_bsp_search_leaf(
  const collision_bsp *bsp,
  bit_vector           breakable_surfaces,
  blam_index_long      leaf_index,
  blam_index_long      plane_index,
  bool                 splits_interior,
  const blam_real3d   *origin,
  const blam_real3d   *delta,
  blam_real            fraction);

/**
 * \brief Tests if \a point is on a surface projected onto a cardinal plane.
 *
 * The edges of the surface must form a convex polygon when projected onto \a plane.
 *
 * \param [in] bsp                The BSP to test against.
 * \param [in] breakable_surfaces The state of breakable surfaces.
 * \param [in] surface_index      The index of the surface to test against.
 * \param [in] plane              The cardinal plane to project onto.
 * \param [in] is_forward_plane   `true` if and only if \a plane is forward-facing 
 *                                with respect to the normal of the source plane.
 * \param [in] point              The point to test.
 *
 * \return `true` if \a point is on the projected surface, otherwise `false`.
 */
BLAM_ATTRIBUTE(noinline)
static
blam_bool collision_surface_test2d(
  const collision_bsp        *bsp,
  bit_vector                  breakable_surfaces,
  blam_index_long             surface_index,
  enum blam_projection_plane  plane,
  blam_bool                   is_forward_plane,
  const blam_real2d          *point);

/**
 * \brief Tests if a vector intersects a surface, without projection.
 *
 * NOTE: THIS FUNCTION IS NOT IN VANILLA HALO.
 *
 * \param [in] bsp                The BSP to test against.
 * \param [in] breakable_surfaces The state of breakable surfaces.
 * \param [in] surface_index      The index of the surface to test against.
 * \param [in] origin             The vector starting point.
 * \param [in] delta              The vector endpoint, relative to \a origin.
 *
 * \return `true` if \a point is on the projected surface, otherwise `false`.
 */
BLAM_ATTRIBUTE(noinline)
static
bool collision_surface_test3d(
  const collision_bsp *bsp,
  bit_vector           breakable_surfaces,
  blam_index_long      surface_index,
  const blam_real3d   *origin,
  const blam_real3d   *delta);  

// -----------------------------------------------------------------------------
// EXPOSED API

blam_index_long blam_collision_bsp_search(
  const collision_bsp *const bsp,
  blam_index_long            root,
  const blam_real3d *const   point)
{
    typedef struct blam_bsp3d_node node_type;
    typedef struct blam_plane3d    plane_type;
    
    const node_type  *const nodes  = BLAM_TAG_BLOCK_BASE(bsp, nodes,  bsp3d_nodes);
    const plane_type *const planes = BLAM_TAG_BLOCK_BASE(bsp, planes, planes);
    
    // if root < 0, it is a leaf index (or -1 if outside of the bsp)
    while (root >= 0) {
      const node_type *const node = &nodes[root];
      const int fwd = blam_plane3d_test(planes + node->plane, point) >= 0.0f;
      root = node->children[fwd];
    }
    
    const blam_index_long leaf_index = blam_sanitize_long_s(root);
    return leaf_index;
}

blam_bool blam_collision_bsp_test_vector(
  const collision_bsp *const bsp,
  const bit_vector           breakable_surfaces,
  const blam_real3d *const   origin,
  const blam_real3d *const   delta,
  blam_real                  max_scale,
  const blam_flags_long      flags, // enum blam_collision_test_flags
  test_vector_result *const  data)
{
  assert(bsp);
  assert(data);

  struct test_vector_context ctx =
  {
    .flags              = flags,
    .bsp                = bsp,
    .breakable_surfaces = breakable_surfaces,
    .origin             = origin,
    .delta              = delta,
    .data               = data,
    .leaf               = -1,
    .leaf_type          = k_bsp_leaf_type_none,
    .plane              = -1,
    .ext                = 
    {
      .just_encountered_leak = false,
      .has_pending_result    = false,
    }
  };
  data->fraction     = fmaxf(max_scale, 0.0f); // Halo doesnt fully clamp here
  data->leaves.count = 0;

  const blam_index_long root = 0;
  const blam_real       start_fraction = 0.0f;
  if (BLAM_UNLIKELY(max_scale < 0.0f))
    max_scale = 0.0f;
  else if (BLAM_UNLIKELY(max_scale > 1.0f))
    max_scale = 1.0f;

  if (collision_bsp_test_vector_node(&ctx, root, start_fraction, max_scale))
    return true;
  else
    return test_vector_context_try_commit_pending_result(&ctx);
}

// -----------------------------------------------------------------------------
// INTERNAL FUNCTIONS

/**
 * \brief Checks if a surface is breakable and broken.
 *
 * \param [in] bsp                The collision BSP.
 * \param [in] breakable_surfaces The state of breakable surfaces.
 * \param [in] surface_index      The index of the surface to test.
 *
 * \return \c true if the surface is broken, otherwise \c false.
 */
BLAM_ATTRIBUTE(pure)
static
bool collision_surface_broken(
  const collision_bsp   *bsp,
  const bit_vector       breakable_surfaces,
  const blam_index_long  surface_index)
{
  const struct blam_collision_surface *const surface = BLAM_TAG_BLOCK_GET(bsp, surface, surfaces, surface_index);
  
  if ((surface->flags & 0x08) != 0 // breakable flag
    && surface->breakable_surface < breakable_surfaces.count
    && !blam_bit_vector_test(&breakable_surfaces, surface->breakable_surface))
      return true; // Surface is breakable and broken.
  
  return false;
}

blam_index_long collision_bsp_search_leaf(
  const collision_bsp *const bsp,
  const bit_vector           breakable_surfaces,
  const blam_index_long      leaf_index,
  const blam_index_long      plane_index,
  const bool                 splits_interior,
  const blam_real3d *const   origin,
  const blam_real3d *const   delta,
  const blam_real            fraction)
{
  assert(bsp);
  assert(origin);
  assert(delta);
  
  blam_real3d terminal = *delta;
  blam_real3d_scale(fraction, &terminal);
  terminal = blam_real3d_add(origin, &terminal);
  
  typedef struct blam_bsp3d_leaf      leaf_type;
  typedef struct blam_bsp2d_reference reference_type;
  
  const leaf_type *const      leaf       = BLAM_TAG_BLOCK_GET(bsp, leaf, leaves, leaf_index);
  const reference_type* const references = BLAM_TAG_BLOCK_BASE(&bsp->bsp2d, references, references) + leaf->first_reference;
  
  // Compute an implicit 2D cardinal basis for the plane (respecting RH coordinates).
  // If projection_inverted is true, then the signs of the basis vectors are flipped.
  const blam_plane3d *const plane                   = BLAM_TAG_BLOCK_GET(bsp, plane, planes, plane_index);
  const enum blam_projection_plane projection_plane = blam_real3d_projection_plane(&plane->normal);
  const bool projection_inverted                    = plane->normal.components[projection_plane] <= 0.0f;
  
  for (const reference_type *ref = references, *end = references + leaf->reference_count
    ; ref < end
    ; ++ref)
  {
    const blam_index_long reference_plane = blam_sanitize_long(ref->plane);
    const bool reference_plane_inverted   = ref->plane < 0;
    
    // NOTE: BSP LEAKS
    // If no BSP2D reference is assigned to plane_index, then we get a BSP leak.
    if (plane_index != reference_plane)
        continue;
    
    // Project terminal onto projection_plane, respecting relative direction of the 
    // BSP2D reference plane.
    const bool is_forward_plane = projection_inverted == reference_plane_inverted;
    const blam_real2d projection = blam_real3d_projected_components(&terminal, projection_plane, is_forward_plane);
    
    // Search the BSP2D for the surface the point lands in.
    const blam_index_long surface_index = blam_bsp2d_search(&bsp->bsp2d, ref->root_node, &projection);
    
    // NOTE: PHANTOM BSP
    // When splits_interior is false, phantom BSP can occur because Halo assumes 
    // that a partitioning plane associated with a BSP2D reference in this leaf is 
    // completely solid within the extents of this leaf. This is true within a 
    // sealed world. If the face of this leaf on this plane extends beyond the 
    // extents of actual surface data (due to improperly placed partitions), then 
    // phantom BSP occurs.
    //
    // Furthermore, if we validate against surface data in infinite precision, we 
    // punch holes into the BSP because phantom BSP can occur over surfaces in 
    // another leaf. 
    if (!splits_interior)
      return surface_index;
    else if (collision_surface_test2d(bsp, breakable_surfaces, surface_index, projection_plane, is_forward_plane, &projection))
      return surface_index; 
  }
  
  // NOTE: If splits_interior is false, then the plane splits the BSP interior
  //       and exterior. Returning -1 in this case indicates a BSP leak, violating 
  //       the sealed world property.
  return -1;
}

blam_index_long blam_bsp2d_search(
  const struct blam_bsp2d *bsp,
  blam_index_long          root,
  const blam_real2d       *point)
{
  assert(bsp);
  assert(point);
  
  while (root >= 0)
  {
    const struct blam_bsp2d_node *const node = BLAM_TAG_BLOCK_GET(bsp, node, nodes, root);
    root = node->children[blam_plane2d_test(&node->plane, point) >= 0.0f];
  }
  
  const blam_index_long surface_index = blam_sanitize_long_s(root);
  return surface_index;
}

blam_bool collision_surface_test2d(
  const collision_bsp        *bsp,
  bit_vector                  breakable_surfaces,
  blam_index_long             surface_index,
  enum blam_projection_plane  plane,
  blam_bool                   is_forward_plane,
  const blam_real2d          *point)
{
  assert(point);
  assert(bsp);
  
  if (collision_surface_broken(bsp, breakable_surfaces, surface_index))
    return false; // Surface is broken; cannot hit
  
  struct blam_collision_surface *const surface = BLAM_TAG_BLOCK_GET(bsp, surface, surfaces, surface_index);
  const blam_index_long first_edge = surface->first_edge;
  
  const blam_pair_int projection = blam_projection_plane_indices(plane, is_forward_plane);
  
  // To test if point is in the bounds of the surface (post-projection),
  // Halo assumes the surface is convex and checks if point is on the 
  // surface-side of each edge. This is done by computing a determinant.
  // Don't let anyone tell you that you need to normalize to do this test.
  blam_index_long next_edge = first_edge;
  do {
    const struct blam_collision_edge *const edge = BLAM_TAG_BLOCK_GET(bsp, edge, edges, next_edge);
    
    const blam_index_long start_index = blam_collision_edge_inorder_vertex(edge, surface_index);
    const blam_index_long end_index   = blam_collision_edge_inorder_vertex_next(edge, surface_index);
    
    const struct blam_collision_vertex *const start = BLAM_TAG_BLOCK_GET(bsp, start, vertices, start_index);
    const struct blam_collision_vertex *const end   = BLAM_TAG_BLOCK_GET(bsp, end, vertices, end_index);
    
    const blam_real2d p0 = {{
      start->point.components[projection.first], 
      start->point.components[projection.second]
    }};
    
    const blam_real2d p1 = {{
      end->point.components[projection.first], 
      end->point.components[projection.second]
    }};
    
    const blam_real2d point_delta = blam_real2d_sub(point, &p0);
    const blam_real2d edge_delta  = blam_real2d_sub(&p1, &p0);
    
    // This is the way Halo performs this test, argument order preserved.
    const blam_real determinant = blam_real2d_det(&point_delta, &edge_delta);
    
    if (determinant > 0.0f)
        return false; // point is outside of surface
    
    next_edge = blam_collision_edge_inorder_edge(edge, surface_index);
  } while (next_edge != first_edge);
  
  return true;
}

bool collision_surface_test3d(
  const collision_bsp *bsp,
  bit_vector           breakable_surfaces,
  blam_index_long      surface_index,
  const blam_real3d   *origin,
  const blam_real3d   *delta)
{
  assert(origin);
  assert(delta);
  
  const struct blam_collision_surface *const surface  = BLAM_TAG_BLOCK_GET(bsp, surface, surfaces, surface_index);
  const struct blam_collision_vertex* const  vertices = BLAM_TAG_BLOCK_BASE(bsp, vertices, vertices);
  const struct blam_collision_edge* const    edges    = BLAM_TAG_BLOCK_BASE(bsp, edges, edges);
  
  if ((surface->flags & 0x08) != 0 // breakable flag
    && surface->breakable_surface < breakable_surfaces.count
    && !blam_bit_vector_test(&breakable_surfaces, surface->breakable_surface))
      return false; // Surface is breakable and broken; surface was not hit.
  
  const blam_index_long first_edge_index             = surface->first_edge;
  const struct blam_collision_edge* const first_edge = &edges[first_edge_index];
  
  const blam_index_long first_vertex_index = blam_collision_edge_inorder_vertex(first_edge, surface_index);
  
  blam_real3d last_vertex = vertices[first_vertex_index].point;
  last_vertex = blam_real3d_sub(&last_vertex, origin);
  
  bool all_signed = true;   // all triple scalar products signed
  bool all_unsigned = true; // all triple scalar products unsigned
  blam_index_long next_edge_index = first_edge_index;
  do
  {
    const struct blam_collision_edge *const edge = &edges[next_edge_index];
    const blam_index_long vertex_index           = blam_collision_edge_inorder_vertex_next(edge, surface_index);
    
    const blam_real3d vertex     = blam_real3d_sub(&vertices[vertex_index].point, origin);
    const blam_real3d edge_bivec = blam_real3d_cross(&last_vertex, &vertex);
    const blam_real   volume     = blam_real3d_dot(delta, &edge_bivec);
    
    all_signed   &= volume <= 0;
    all_unsigned &= volume >= 0;
    
    next_edge_index = blam_collision_edge_inorder_edge(edge, surface_index);
    last_vertex     = vertex;
  } while (next_edge_index != first_edge_index);
  
  return all_signed || all_unsigned;
}

blam_bool test_vector_context_try_commit_result(
  struct test_vector_context *ctx,
  blam_real       fraction,
  blam_index_long plane_index,
  blam_index_long surface_index)
{
  assert(ctx);
  assert(plane_index != -1);
  
  if (surface_index == -1)
    return false;
  
  const struct blam_collision_surface *const surface = BLAM_TAG_BLOCK_GET(ctx->bsp, surface, surfaces, surface_index);
  
  const bool test_invisible_surfaces = (ctx->flags & k_collision_test_ignore_invisible_surfaces) == 0;
  const bool test_breakable_surfaces = (ctx->flags & k_collision_test_ignore_breakable_surfaces) == 0;
  if (((surface->flags & 0x02) != 0 && !test_invisible_surfaces)
    || ((surface->flags & 0x08) != 0 && !test_breakable_surfaces))
    return false;
  
  ctx->data->fraction   = fraction;
  ctx->data->last_split = BLAM_TAG_BLOCK_GET(ctx->bsp, struct blam_plane3d*, planes, plane_index);
  ctx->data->surface.index             = surface_index;
  ctx->data->surface.plane             = surface->plane;
  ctx->data->surface.flags             = surface->flags;
  ctx->data->surface.breakable_surface = surface->breakable_surface;
  ctx->data->surface.material          = surface->material;
  
  return true;
}

blam_bool test_vector_context_try_commit_pending_result(
  struct test_vector_context *ctx)
{
  assert(ctx);
  
  if (!ctx->ext.has_pending_result)
    return false;
  
  return test_vector_context_try_commit_result(
    ctx, 
    ctx->ext.pending.fraction,
    ctx->ext.pending.plane,
    ctx->ext.pending.surface);
}

blam_bool collision_bsp_test_vector_node(
  struct test_vector_context *const ctx,
  const blam_index_long             root,
  const blam_real                   fraction,
  const blam_real                   terminal)
{
  if (BLAM_UNLIKELY(root < 0))
  {
    const blam_index_long leaf = blam_sanitize_long_s(root);
    return collision_bsp_test_vector_leaf(ctx, leaf, fraction);
  }
  
  const struct blam_bsp3d_node *const node  = BLAM_TAG_BLOCK_GET(ctx->bsp, node, bsp3d_nodes, root);
  const struct blam_plane3d *const    plane = BLAM_TAG_BLOCK_GET(ctx->bsp, plane, planes, node->plane);
  
  // We need to test the current point as well as the terminal point 
  // against the plane given by node->plane.
  // If both are on the same side of the plane, then we simply go down
  // that part of the tree in one recursive call.
  // If they land on different sides of the plane, then we may need to 
  // recurse down both sides of the tree.
  // Halo performs these tests in the following steps:
  const blam_real test_origin   = blam_plane3d_test(plane, ctx->origin);
  const blam_real dot_delta     = blam_real3d_dot(&plane->normal, ctx->delta);
  const blam_real point_test    = test_origin + fraction * dot_delta;
  const blam_real terminal_test = test_origin + terminal * dot_delta;
  const bool any_before = (point_test < 0.0f) || (terminal_test < 0.0f);
  const bool any_after  = (point_test >= 0.0f) || (terminal_test >= 0.0f);
  
  if (!any_before || !any_after) {
    // The origin and terminal points are on the same side of the tree.
    // Just recurse down the subtree those points are on.
    const blam_index_long new_root = node->children[any_after ? 1 : 0];
    return collision_bsp_test_vector_node(ctx, new_root, fraction, terminal);
  } else {
    // The origin and terminal points are on opposite sides of the plane.
    // <n, delta> < 0 if and only if the point given by fraction is in front
    // of the plane (point_test >= 0).
    // This comparison is retained as is from Halo.
    const bool plane_faces_forward = !(dot_delta >= 0.0f);
    const blam_index_long first_child  = node->children[plane_faces_forward ? 1 : 0];
    const blam_index_long second_child = node->children[plane_faces_forward ? 0 : 1];

    // intersection is the scalar t such that:
    //  <n, origin + t * delta> - w = 0,
    //  i.e. t = -(<n, origin> - w)/<n, delta>
    // where origin is the vector origin, delta is the vector delta,
    // and (n,w) describes the plane
    //
    // The condition to get here is that the points given by fraction
    // and terminal_fraction are on opposite sides of the plane.
    // If we manage to get here, then <n, delta> != 0.
    // We can therefore divide by <n, delta>.
    const blam_real intersection = -(test_origin / dot_delta);
    
    if (collision_bsp_test_vector_node(ctx, first_child, fraction, intersection)) {
      // Found an intersection in the first child subtree.
      return true;
    } else if (BLAM_UNLIKELY(ctx->data->fraction <= intersection)) {
      // An intersection occurred before the splitting plane between children.
      return false;
    } else {
      // Continue to test along the second child subtree.
      ctx->plane = node->plane;
      return collision_bsp_test_vector_node(ctx, second_child, intersection, terminal);
    }
  }
}

/**
 * \brief Internal; common subroutine used in #collision_bsp_test_vector_leaf.
 *
 * It is assumed that, if \a splits_interior is \c false, then `ctx->plane` splits 
 * BSP interior and exterior; `ctx->plane` is a solid partition within the extents 
 * of the leaf.
 *
 * \param [in,out] ctx         The test context.
 * \param [in] leaf            The leaf being tested.
 * \param [in] fraction        The distance to the point of intersection with the 
 *                             leaf, as a fraction of `ctx->delta`.
 * \param [in] splits_interior If \c true, the `ctx->plane` divides two BSP 
 *                             interior leaves.
 * \param [in] commit_result   If \c true, the surface found (if any) should be 
 *                             committed or verified. 
 *
 * \return \c true if a surface was intersected, otherwise \c false.
 */
static
bool collision_bsp_test_vector_leaf_visit_surface(
  struct test_vector_context *const ctx,
  const blam_index_long             leaf_index,
  blam_real                         fraction,
  const bool                        splits_interior,
  
  // NON-VANILLA PARAMETERS
  bool commit_result)
{
  if (leaf_index == -1)
      return false;
  
  // true if the vector is testing the front of the surface found (if any)
  const bool frontfacing = blam_bsp_leaf_type_interior(ctx->leaf_type);
  
  blam_index_long plane_index = ctx->plane;
  blam_index_long surface_index = collision_bsp_search_leaf(
    ctx->bsp,
    ctx->breakable_surfaces,
    leaf_index,
    plane_index,
    splits_interior,
    ctx->origin,
    ctx->delta,
    fraction);
  
  // true when a BSP leak is encountered on ctx->plane in this leaf
  bool leak_encountered = !splits_interior && surface_index == -1;
  
  // TODO: MITIGATE BSP LEAK HERE
  // if (leak_encountered)
  //   ... update surface_index if a surface was found

  // If there is a pending intersection result, reject it if a leak was encountered.
  // The copied reassignment to leak_encountered is intentional, to keep the value 
  // updated when BSP leak mitigations are in place.
  leak_encountered = !splits_interior && surface_index == -1;
  if (BLAM_UNLIKELY(leak_encountered && ctx->ext.has_pending_result))
  {
    ctx->ext.has_pending_result = false;
  }
  
  const bool may_require_validation = !splits_interior;
  if (surface_index == -1)
  {
    // DO NOTHING
  } else if (ctx->ext.has_pending_result)
  {
    // Pending result was confirmed by the current surface result.
    // Replace the current result with the pending result.
    fraction      = ctx->ext.pending.fraction;
    plane_index   = ctx->ext.pending.plane;
    surface_index = ctx->ext.pending.surface;
  } else if (!commit_result)
  {
    // Discard this intersection.
    surface_index = -1;
  } else if (mitigate_phantom_bsp && may_require_validation) {
    const bool validated = collision_surface_test3d(
      ctx->bsp,
      ctx->breakable_surfaces,
      surface_index,
      ctx->origin,
      ctx->delta);
    
    if (validated)
    {
      // NOTHING TO DO, SURFACE IS OK
    } else if (frontfacing)
    {
      // Make this intersection result the pending result.
      ctx->ext.pending.fraction = fraction;
      ctx->ext.pending.plane    = plane_index;
      ctx->ext.pending.surface  = surface_index;
      surface_index = -1;
    } else {
      // Potential phantom BSP surface is backfacing.
      // In this case, we reject the result as phantom BSP if the last solid 
      // partition was a leak.
      if (ctx->ext.just_encountered_leak)
        surface_index = -1;
    }
  } else {
    // NOTHING TO DO, SURFACE IS OK
  }
  
  ctx->ext.just_encountered_leak = leak_encountered;
  return test_vector_context_try_commit_result(ctx, fraction, plane_index, surface_index);
}

blam_bool collision_bsp_test_vector_leaf(
  struct test_vector_context *const ctx,
  const blam_index_long             leaf,
  const blam_real                   fraction)
{
  const enum blam_bsp_leaf_type leaf_type = blam_collision_bsp_classify_leaf(ctx->bsp, leaf);
  BLAM_ASSUME(k_bsp_leaf_type_none <= leaf_type && leaf_type <= k_bsp_leaf_type_exterior);
  BLAM_ASSUME(k_bsp_leaf_type_none <= ctx->leaf_type && ctx->leaf_type <= k_bsp_leaf_type_exterior);
  
  const bool test_frontfacing = (ctx->flags & k_collision_test_front_facing_surfaces) != 0;
  const bool test_backfacing  = (ctx->flags & k_collision_test_back_facing_surfaces) != 0;
  
  // PHANTOM BSP MITIGATIONS:
  // If we are mitigating phantom BSP, then we need to test both front- and 
  // back-facing surfaces to observe BSP leaks. The result is only committed if its 
  // a surface the user actually desires to test.
  if ((test_frontfacing || mitigate_phantom_bsp)
    && blam_bsp_leaf_type_interior(ctx->leaf_type)
    && leaf_type == k_bsp_leaf_type_exterior)
  {
    // Testing front-facing surfaces
    // Plane splits BSP interior at ctx->leaf from BSP exterior at leaf.
    const blam_index_long tested_leaf     = ctx->leaf;
    const bool            splits_interior = false;
    const bool            commit_result   = test_frontfacing;
    
    const bool result = collision_bsp_test_vector_leaf_visit_surface(
      ctx, 
      tested_leaf, 
      fraction, 
      splits_interior, 
      commit_result);
    if (BLAM_LIKELY(result))
      return true;
  } else if ((test_backfacing || mitigate_phantom_bsp)
    && ctx->leaf_type == k_bsp_leaf_type_exterior
    && blam_bsp_leaf_type_interior(leaf_type))
  {
    // Testing back-facing surfaces
    // Plane splits BSP exterior at ctx->leaf from BSP interior at leaf.
    const blam_index_long tested_leaf     = leaf;
    const bool            splits_interior = false;
    const bool            commit_result   = test_backfacing;
    
    const bool result = collision_bsp_test_vector_leaf_visit_surface(
      ctx, 
      tested_leaf, 
      fraction, 
      splits_interior, 
      commit_result);
    if (BLAM_LIKELY(result))
      return true;
  } else if ((ctx->flags & k_collision_test_ignore_two_sided_surfaces) == 0
    && ctx->leaf_type == k_bsp_leaf_type_double_sided
    && leaf_type == k_bsp_leaf_type_double_sided)
  {
    // Testing double-sided surfaces
    // Plane splits BSP interior leaves at ctx->leaf and leaf.
    const blam_index_long tested_leaf
      = (ctx->flags & k_collision_test_front_facing_surfaces) != 0 ? ctx->leaf
                                                                   : leaf;
    const bool splits_interior = true;
    const bool commit_result   = true;

    const bool result = collision_bsp_test_vector_leaf_visit_surface(
      ctx, 
      tested_leaf, 
      fraction, 
      splits_interior, 
      commit_result);
    if (result)
      return true;
    // NOTE: Not a sealed-world violation; double-sided surface may be breakable.
  } else
  {
    // DO NOTHING
  }

  // ------------------------------
  // Record the leaf into the query
  if (leaf != -1)
  {
    // NOTE: branchless code is a pessimization here.
    if (BLAM_LIKELY(ctx->data->leaves.count < 0x100))
      ctx->data->leaves.stack[ctx->data->leaves.count++] = leaf;
    else
      ctx->data->leaves.stack[0x100 - 1] = leaf;
  }
  
  ctx->leaf      = leaf;
  ctx->leaf_type = leaf_type;
  
  return false;
}

