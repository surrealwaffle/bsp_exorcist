#include "blam/math.h"
#include <assert.h>
#include <math.h>

static double blam_epsilon_double = 0x1.A36E2Ep-14;

BLAM_ATTRIBUTE(pure)
static
blam_real blam_dot(const blam_real *u, const blam_real *v, int dim)
{
  assert(u);
  assert(v);
  assert(dim >= 1);
    
  // annoyingly, this generates better asm than the generic way of writing it
  // dot product is pervasive, so im willing to write this blight
  if (dim == 1) {
    return u[0] * v[0];
  } else if (dim == 2) {
    return u[0] * v[0] + u[1] * v[1];
  } else if (dim == 3) {
    // This associativity produces slightly better assembly.
    return u[0] * v[0] + (u[1] * v[1] + u[2] * v[2]);
  } else {
    blam_real result = 0.0f;
    for (int i = 0; i < dim; ++i)
      result += u[i] * v[i];
    return result;
  }
}

static
void blam_scale(blam_real scalar, blam_real *v, int dim)
{
  assert(v);
  assert(dim >= 1);
  
  for (int i = 0; i < dim; ++i)
    v[i] *= scalar;
}

static
void blam_vadd(const blam_real *u, const blam_real *v, blam_real *out, int dim)
{
  assert(u && v && out);
  assert(dim >= 1);
   
  for (int i = 0; i < dim; ++i)
    out[i] = u[i] + v[i];
}

static
void blam_vsub(const blam_real *u, const blam_real *v, blam_real *out, int dim)
{
  assert(u && v && out);
  assert(dim >= 1);
  
  for (int i = 0; i < dim; ++i)
  out[i] = u[i] - v[i];
}

BLAM_ATTRIBUTE(pure)
static
blam_real blam_norm2(const blam_real *v, int dim)
{
  assert(v);
  assert(dim >= 1);
  
  blam_real result = blam_dot(v, v, dim);
    return result;
}

BLAM_ATTRIBUTE(pure)
static
blam_real blam_norm(const blam_real *v, int dim)
{
  assert(v);
  assert(dim >= 1);
  
  return sqrtf(blam_norm2(v, dim));
}

static
blam_real blam_normalize(blam_real *v, int dim)
{
  assert(v);
  assert(dim >= 1);
  
  const blam_real norm = blam_norm(v, dim);
  if (norm < blam_epsilon_double)
    return 0.0f;
    
  blam_scale(1.0f / norm, v, dim);
  return norm;
}

blam_real blam_real3d_dot(const blam_real3d *u, const blam_real3d *v)
{
  return blam_dot(u->components, v->components, 3);
}

blam_real blam_real2d_dot(const blam_real2d *u, const blam_real2d *v)
{
  return blam_dot(u->components, v->components, 2);
}

blam_real blam_real3d_norm(const blam_real3d *v)
{
  return blam_norm(v->components, 3);
}

blam_real blam_real2d_norm(const blam_real2d *v)
{
  return blam_norm(v->components, 2);
}

blam_real blam_real3d_normalize(blam_real3d *v)
{
  return blam_normalize(v->components, 3);
}

blam_real blam_real2d_normalize(blam_real2d *v)
{
  return blam_normalize(v->components, 2);
}

void blam_real3d_scale(blam_real s, blam_real3d *v)
{
  blam_scale(s, v->components, 3);
}

void blam_real2d_scale(blam_real s, blam_real2d *v)
{
  blam_scale(s, v->components, 2);
}

blam_real3d blam_real3d_add(const blam_real3d *u, const blam_real3d *v)
{
  blam_real3d result;
  blam_vadd(u->components, v->components, result.components, 3);
  return result;
}

blam_real2d blam_real2d_add(const blam_real2d *u, const blam_real2d *v)
{
  blam_real2d result;
  blam_vadd(u->components, v->components, result.components, 2);
  return result;
}

blam_real3d blam_real3d_sub(const blam_real3d *u, const blam_real3d *v)
{
  blam_real3d result;
  blam_vsub(u->components, v->components, result.components, 3);
  return result;
}

blam_real2d blam_real2d_sub(const blam_real2d *u, const blam_real2d *v)
{
  blam_real2d result;
  blam_vsub(u->components, v->components, result.components, 2);
  return result;
}

blam_real3d blam_real3d_cross(const blam_real3d *u, const blam_real3d *v)
{
  assert(u);
  assert(v);
  const blam_real3d result = {
    u->components[1] * v->components[2] - u->components[2] * v->components[1],
    u->components[2] * v->components[0] - u->components[0] * v->components[2],
    u->components[0] * v->components[1] - u->components[1] * v->components[0]
  };
  return result;
}

blam_real blam_real2d_det(const blam_real2d *u, const blam_real2d *v)
{
  assert(u);
  assert(v);
  return u->components[0] * v->components[1] - u->components[1] * v->components[0];
}

blam_real blam_plane3d_test(const blam_plane3d *plane, const blam_real3d *v)
{
  assert(plane);
  assert(v);
  return blam_real3d_dot(&plane->normal, v) - plane->d;
}

blam_real blam_plane2d_test(const blam_plane2d *plane, const blam_real2d *v)
{
  assert(plane);
  assert(v);
  return blam_real2d_dot(&plane->normal, v) - plane->d;
}

blam_real blam_plane3d_test_implicit(
  const blam_plane3d *const plane,
  const blam_real3d  *const origin,
  const blam_real3d  *const delta,
  const blam_real           scale)
{
  return blam_plane3d_test(plane, origin)
    + scale * blam_real3d_dot(&plane->normal, delta);
}

blam_real blam_plane2d_test_implicit(
  const blam_plane2d *const plane,
  const blam_real2d  *const origin,
  const blam_real           scale,
  const blam_real2d  *const delta)
{
  return blam_plane2d_test(plane, origin)
    + scale * blam_real2d_dot(&plane->normal, delta);
}

blam_real3d blam_plane3d_project(const blam_plane3d *plane, const blam_real3d *v)
{
  blam_real3d orthogonal_component = plane->normal;
  blam_real3d_scale(blam_plane3d_test(plane, v), &orthogonal_component);
  
  return blam_real3d_sub(v, &orthogonal_component);
}

blam_real2d blam_plane2d_project(const blam_plane2d *plane, const blam_real2d *v)
{
  blam_real2d orthogonal_component = plane->normal;
  blam_real2d_scale(blam_plane2d_test(plane, v), &orthogonal_component);

  return blam_real2d_sub(v, &orthogonal_component);
}

blam_long blam_maxl(const blam_long a, const blam_long b)
{
  return a < b ? b : a;
}

blam_short blam_maxs(const blam_short a, const blam_short b)
{
  return a < b ? b : a;
}

blam_byte blam_maxb(const blam_byte a, const blam_byte b)
{
  return a < b ? b : a;
}

blam_real blam_maxf(const blam_real a, const blam_real b)
{
  return a < b ? b : a;
}

blam_long blam_minl(const blam_long a, const blam_long b)
{
  return a < b ? a : b;
}

blam_short blam_mins(const blam_short a, const blam_short b)
{
  return a < b ? a : b;
}

blam_byte blam_minb(const blam_byte a, const blam_byte b)
{
  return a < b ? a : b;
}

blam_real blam_minf(const blam_real a, const blam_real b)
{
  return a < b ? a : b;
}

blam_real blam_clampf(const blam_real v, const blam_real a, const blam_real b)
{
  return b < v ? b : v < a ? a : v;
}

/*
static blam_pair_short projection_indices_lookup[6] = {
  [2*k_blam_projection_plane_yz + k_blam_plane_invert]  = {2, 1},
  [2*k_blam_projection_plane_yz + k_blam_plane_forward] = {1, 2},
  
  [2*k_blam_projection_plane_xz + k_blam_plane_invert]  = {0, 2},
  [2*k_blam_projection_plane_xz + k_blam_plane_forward] = {2, 0},
  
  [2*k_blam_projection_plane_xy + k_blam_plane_invert]  = {1, 0},
  [2*k_blam_projection_plane_xy + k_blam_plane_forward] = {0, 1},
};
*/

enum blam_projection_plane blam_real3d_projection_plane(const blam_real3d *v)
{
  const blam_real x = fabsf(v->components[0]);
  const blam_real y = fabsf(v->components[1]);
  const blam_real z = fabsf(v->components[2]);

  if (z >= y && z >= x)
    return k_blam_projection_plane_xy;
  else if (y >= x && y >= z)
    return k_blam_projection_plane_xz;
  else
    return k_blam_projection_plane_yz;
}

static int my_small_mod3(int x)
{
  return x < 3 ? x : x - 3;
}

blam_pair_int blam_projection_plane_indices(
  enum blam_projection_plane plane,
  blam_bool                  is_forward_plane)
{    
  assert(plane >= k_blam_projection_plane_yz
    && plane <= k_blam_projection_plane_xy);
  assert(is_forward_plane == !!is_forward_plane);

  // Halo uses a lookup-table for this operation. See above for the LUT.
  // The following is equivalent.  
  blam_pair_int result = {
    .first  = my_small_mod3(2 - is_forward_plane + plane),
    .second = my_small_mod3(1 + is_forward_plane + plane)
  };
  
  return result;
}

blam_real2d blam_real3d_projected_components(
  const blam_real3d *restrict v,
  enum blam_projection_plane  plane,
  blam_bool                   is_forward_plane)
{
  assert(v);
  
  blam_index_short ppi, spi;
  blam_pair_int i = blam_projection_plane_indices(plane, is_forward_plane);
  blam_real2d result = {v->components[i.first], v->components[i.second]};
  
  return result;
}