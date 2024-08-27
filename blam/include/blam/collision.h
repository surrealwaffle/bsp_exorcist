#ifndef BLAM_COLLISION_BSP_H
#define BLAM_COLLISION_BSP_H

#include <stdbool.h>

#include "base.h"
#include "math.h"
#include "tag.h"

////////////////////////////////////////////////////////////////////////////////
// Tags

/**
 * \brief The various BSP leaf categories.
 */
enum blam_bsp_leaf_type
{
  k_bsp_leaf_type_none = 0,     ///< No leaf.
  k_bsp_leaf_type_interior,     ///< BSP interior with no double-sided surfaces.
  k_bsp_leaf_type_double_sided, ///< BSP interior with double-sided surfaces.
  k_bsp_leaf_type_exterior      ///< BSP exterior volume.
};

struct blam_bsp3d_node
{
  blam_index_long plane;
  blam_index_long children[2];
}; BLAM_ASSERT_SIZE(struct blam_bsp3d_node, 0x0C);

struct blam_bsp3d_leaf
{
  blam_flags_short flags; // might be blam_flags_byte
  blam_short       reference_count;
  blam_index_long  first_reference;
}; BLAM_ASSERT_SIZE(struct blam_bsp3d_leaf, 0x08);

struct blam_bsp2d_reference
{
  blam_index_long plane;
  blam_index_long root_node;
}; BLAM_ASSERT_SIZE(struct blam_bsp2d_reference, 0x08);

struct blam_bsp2d_node
{
  blam_plane2d    plane;
  blam_index_long children[2];
}; BLAM_ASSERT_SIZE(struct blam_bsp2d_node, 0x14);

struct blam_bsp2d
{
  struct blam_tag_block references; // struct blam_bsp2d_reference
  struct blam_tag_block nodes;      // struct blam_bsp2d_node
}; BLAM_ASSERT_SIZE(struct blam_bsp2d, 0x18);

struct blam_collision_surface
{
  blam_index_long  plane;
  blam_index_long  first_edge;
  blam_flags_byte  flags;
  blam_index_byte  breakable_surface;
  blam_index_short material;
}; BLAM_ASSERT_SIZE(struct blam_collision_surface, 0x0C);

struct blam_collision_edge
{
  // Given a surface index s, if surfaces[i] == s, then:
  //   1. vertices[i] is the first oriented vertex for s along this edge
  //   2. s[i] is the next edge to follow about the surface

  blam_index_long vertices[2];
  blam_index_long edges[2];
  blam_index_long surfaces[2];
}; BLAM_ASSERT_SIZE(struct blam_collision_edge, 0x18);

struct blam_collision_vertex
{
  struct blam_real3d point;
  blam_index_long    first_edge;
}; BLAM_ASSERT_SIZE(struct blam_collision_vertex, 0x10);

struct blam_collision_bsp
{
  struct blam_tag_block bsp3d_nodes; // struct blam_bsp3d_node
  struct blam_tag_block planes;      // struct blam_plane3d
  struct blam_tag_block leaves;      // struct blam_bsp3d_leaf
  struct blam_bsp2d     bsp2d;
  struct blam_tag_block surfaces;    // struct blam_collision_surface
  struct blam_tag_block edges;       // struct blam_collision_edge
  struct blam_tag_block vertices;    // struct blam_collision_vertex
}; BLAM_ASSERT_SIZE(struct blam_collision_bsp, 0x60);

////////////////////////////////////////////////////////////////////////////////
// Engine

/**
 * \brief Flags that control collision and intersection-testing behaviour.
 */
enum blam_collision_test_flags
{
  // If both of these flags are clear, they default to set.
  k_collision_test_front_facing_surfaces = 1L << 0,
  k_collision_test_back_facing_surfaces  = 1L << 1,

  k_collision_test_ignore_two_sided_surfaces = 1L << 2,
  k_collision_test_ignore_invisible_surfaces = 1L << 3,
  k_collision_test_ignore_breakable_surfaces = 1L << 4,

  // The bits relevant for collision testing between a vector and a BSP.
  k_collision_test_bsp_bits = k_collision_test_front_facing_surfaces
                            | k_collision_test_back_facing_surfaces
                            | k_collision_test_ignore_two_sided_surfaces
                            | k_collision_test_ignore_invisible_surfaces
                            | k_collision_test_ignore_breakable_surfaces,

  k_collision_test_structure = 1L << 5,
  k_collision_test_media     = 1L << 6,
  k_collision_test_objects   = 1L << 7,
  k_collision_test_all_categories = k_collision_test_structure
                                  | k_collision_test_media
                                  | k_collision_test_objects,

  // If all of these flags are clear, they default to set.
  k_collision_test_bipeds         = 1L << 8,
  k_collision_test_vehicles       = 1L << 9,
  k_collision_test_weapons        = 1L << 10,
  k_collision_test_equipment      = 1L << 11,
  k_collision_test_garbage        = 1L << 12,
  k_collision_test_projectiles    = 1L << 13,
  k_collision_test_scenery        = 1L << 14,
  k_collision_test_machines       = 1L << 15,
  k_collision_test_controls       = 1L << 16,
  k_collision_test_light_fixtures = 1L << 17,
  k_collision_test_placeholders   = 1L << 18,
  k_collision_test_sound_scenery  = 1L << 19,

  k_collision_test_try_to_keep_location_valid = 1L << 20,

  // Probably has to do with bipeds in vehicles, need to test and reverse
  k_collision_test_skip_passthrough_bipeds = 1L << 21,

  // If set, when testing against a vehicle, the vehicle mass-spheres are used
  // instead of the vehicle's collision model.
  k_collision_test_use_vehicle_physics = 1L << 22
};

struct blam_collision_surface_result
{
  blam_index_long  index;             ///< The index of the surface.
  blam_index_long  plane;             ///< The index of the surface plane.
  blam_flags_byte  flags;             ///< The surface flags.
  blam_index_byte  breakable_surface; ///< The breakable surface index.
  blam_index_short material;          ///< The surface material type.
};

struct blam_collision_bsp_test_vector_result
{
  blam_real     fraction;   ///< The relative distance to the earliest intersection.
  blam_plane3d *last_split; ///< The splitting plane of the earliest intersection.

  struct blam_collision_surface_result surface; ///< The intersected surface.

  struct {
    blam_long       count;        ///< The number of leaves populating #stack.
    blam_index_long stack[0x100]; ///< The stack of BSP leaf indices visited.
  } leaves;
}; BLAM_ASSERT_SIZE(struct blam_collision_bsp_test_vector_result, 0x418);

/**
 * \brief Finds the leaf of a collision BSP containing \a point.
 *
 * \param [in] bsp   The collision BSP to search through.
 * \param [in] root  The root node index of the subtree to search in.
 *                   Supply as `0` to search the entire tree.
 * \param [in] point The point to search for.
 *
 * \return The index of the leaf containing \a point, or 
 *         `-1` if \a point is not enclosed within \a bsp.
 */
blam_index_long blam_collision_bsp_search(
  const struct blam_collision_bsp *bsp,
  blam_index_long                  root,
  const blam_real3d               *point);

/**
 * \brief Searches a BSP2D subtree for the surface containing a point.
 *
 * \param [in] bsp   The BSP2D to search.
 * \param [in] root  The root node index of the subtree to search in.
 * \param [in] point The point to search for.
 *
 * \return The index of the surface containing \a point.
 */
blam_index_long blam_bsp2d_search(
  const struct blam_bsp2d *bsp,
  blam_index_long          root,
  const blam_real2d       *point);

/**
 * \brief Tests a vector against a collision BSP.
 *
 * \a max_scale is clamped to the interval `[0.0, 1.0]`.
 *
 * \param [in]  bsp                The collision BSP to test against.
 * \param [in]  breakable_surfaces The breakable surfaces state.
 * \param [in]  origin             The starting point of the vector.
 * \param [in]  delta              The vector endpoint, relative to \a origin.
 * \param [in]  max_scale          The proportional distance of \a origin to search.
 * \param [in]  flags              See `enum blam_collision_test_flags`.
 * \param [out] data               Receives the intersection result.
 *
 * \return `true` if an intersection occurred, otherwise `false`.
 */
blam_bool blam_collision_bsp_test_vector(
  const struct blam_collision_bsp *bsp,
  struct blam_bit_vector           breakable_surfaces,
  const blam_real3d               *origin,
  const blam_real3d               *delta,
  blam_real                        max_scale,
  blam_flags_long                  flags, // enum blam_collision_test_flags
  struct blam_collision_bsp_test_vector_result *data);

/** 
 * \brief Classifies a collision BSP leaf.
 *
 * \param [in] bsp        The collision BSP.
 * \param [in] leaf_index The index of the leaf to classify.
 *
 * \return The leaf classification.
 */
static inline
enum blam_bsp_leaf_type blam_collision_bsp_classify_leaf(
  const struct blam_collision_bsp *const bsp,
  const blam_index_long            leaf_index)
{
  if (leaf_index != -1) {
    const struct blam_bsp3d_leaf *leaf = BLAM_TAG_BLOCK_GET(bsp, leaf, leaves, leaf_index);
    return (leaf->flags & 0x01) ? k_bsp_leaf_type_double_sided
                                : k_bsp_leaf_type_interior;
  } else {
    return k_bsp_leaf_type_exterior;
  }
}

/**
 * \brief Tests if a BSP leaf type indicates an interior leaf node.
 *
 * \param [in] leaf_type The leaf type to test.
 *
 * \return \c true if \a leaf_type indicates a leaf on the interior of a BSP.
 */
static inline
bool blam_bsp_leaf_type_interior(const enum blam_bsp_leaf_type type)
{
  return type == k_bsp_leaf_type_interior || type == k_bsp_leaf_type_double_sided;
}

#endif // BLAM_COLLISION_BSP_H