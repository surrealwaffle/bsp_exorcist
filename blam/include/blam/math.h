#ifndef BLAM_MATH_H
#define BLAM_MATH_H

#include <math.h>
#include <tgmath.h>

#include "platform.h"
#include "base.h"

// In general, if restrict would be needed in certain cases, a function would
// instead simply return by value. This generates assembly close to what Halo 
// has anyway.

/**
 * \brief Substitute for the 80-bit extended double precision floating point that 
 *        Halo uses through the x87 coprocessor.
 *
 * When compiled for modern processors, the x87 coprocessor typically gets passed 
 * over in favor of SSE registers. This type is used where appropriate to maintain 
 * similar degreees of precision between this library and Halo.
 *
 * Note that `blam_real` is correct with respect to storage representation.
 */
typedef double blam_real_highp;

#define BLAM_REAL_EPSILON 0x1.A36E2Ep-14

/**
 * \brief Returns the scalar product between \a u and \a v.
 */
static inline
blam_real_highp blam_real3d_dot(const blam_real3d *u, const blam_real3d *v)
{
  const blam_real* uc = &u->components[0];
  const blam_real* vc = &v->components[0];
  return (blam_real_highp)uc[0] * (blam_real_highp)vc[0]
    + (blam_real_highp)uc[1] * (blam_real_highp)vc[1]
    + (blam_real_highp)uc[2] * (blam_real_highp)vc[2];
}

/**
 * \brief Returns the scalar product between \a u and \a v.
 */
static inline
blam_real_highp blam_real2d_dot(const blam_real2d *u, const blam_real2d *v)
{
  const blam_real* uc = &u->components[0];
  const blam_real* vc = &v->components[0];
  return (blam_real_highp)uc[0] * (blam_real_highp)vc[0]
    + (blam_real_highp)uc[1] * (blam_real_highp)vc[1];
}

/**
 * \brief Returns the magnitude of \a v.
 */
static inline
blam_real_highp blam_real3d_norm(const blam_real3d *v)
{
  return sqrt(blam_real3d_dot(v, v));
}

/**
 * \brief Returns the magnitude of \a v.
 */
static inline
blam_real_highp blam_real2d_norm(const blam_real2d *v) 
{
  return sqrt(blam_real2d_dot(v, v));
}

/**
 * \brief Scales a vector \a v by scalar \a s.
 */
static inline
void blam_real3d_scale(blam_real s, blam_real3d *v)
{
  for (int i = 0; i < 3; ++i)
    v->components[i] = v->components[i] * s;
}

/**
 * \brief Scales a vector \a v by scalar \a s.
 */
static inline
void blam_real3d_scale_highp(blam_real_highp s, blam_real3d *v)
{
  for (int i = 0; i < 3; ++i)
    v->components[i] = (blam_real)(v->components[i] * s);
}

/**
 * \brief Scales a vector \a v by scalar \a s.
 */
static inline
void blam_real2d_scale(blam_real s, blam_real2d *v)
{
  for (int i = 0; i < 2; ++i)
    v->components[i] = v->components[i] * s;
}

/**
 * \brief Scales a vector \a v by scalar \a s.
 */
static inline
void blam_real2d_scale_highp(blam_real_highp s, blam_real2d *v)
{
  for (int i = 0; i < 2; ++i)
    v->components[i] = (blam_real)(v->components[i] * s);
}

/**
 * \brief Normalizes vector \a v.
 *
 * \param [in,out] v The vector to normalize.
 *
 * \return The magnitude of \a v before normalization.
 */
static inline
blam_real_highp blam_real3d_normalize(blam_real3d *v)
{
  const blam_real_highp norm = blam_real3d_norm(v);
  if (norm < BLAM_REAL_EPSILON)
    return (blam_real_highp)0.0;
  
  blam_real3d_scale((blam_real_highp)1.0 / norm, v);
  return norm;
}

/**
 * \brief Normalizes vector \a v.
 *
 * \param [in,out] v The vector to normalize.
 *
 * \return The magnitude of \a v before normalization.
 */
static inline
blam_real_highp blam_real2d_normalize(blam_real2d *v)
{
  const blam_real_highp norm = blam_real2d_norm(v);
  if (norm < BLAM_REAL_EPSILON)
    return (blam_real_highp)0.0;
  
  blam_real2d_scale_highp((blam_real_highp)1.0 / norm, v);
  return norm;
}

/**
 * \brief Returns the sum of vectors \a u and \a v.
 */
static inline
blam_real3d blam_real3d_add(const blam_real3d *u, const blam_real3d *v)
{
  blam_real3d result;
  for (int i = 0; i < 3; ++i)
  {
    result.components[i] = (blam_real)((blam_real_highp)u->components[i] + (blam_real_highp)v->components[i]);
  }
  return result;
}

/**
 * \brief Returns the sum of vectors \a u and \a v.
 */
static inline
blam_real2d blam_real2d_add(const blam_real2d *u, const blam_real2d *v)
{
  blam_real2d result;
  for (int i = 0; i < 2; ++i)
  {
    result.components[i] = (blam_real)((blam_real_highp)u->components[i] + (blam_real_highp)v->components[i]);
  }
  return result;
}

/**
 * \brief Returns the difference between vectors \a u and \a v.
 */
static inline
blam_real3d blam_real3d_sub(const blam_real3d *u, const blam_real3d *v)
{
  blam_real3d result;
  for (int i = 0; i < 3; ++i)
  {
    result.components[i] = (blam_real)((blam_real_highp)u->components[i] - (blam_real_highp)v->components[i]);
  }
  return result;
}

/**
 * \brief Returns the difference between vectors \a u and \a v.
 */
static inline
blam_real2d blam_real2d_sub(const blam_real2d *u, const blam_real2d *v)
{
  blam_real2d result;
  for (int i = 0; i < 2; ++i)
  {
    result.components[i] = (blam_real)((blam_real_highp)u->components[i] - (blam_real_highp)v->components[i]);
  }
  return result;
}

/**
 * \brief Returns the vector product between \a u and \a v.
 */
static inline
blam_real3d blam_real3d_cross(const blam_real3d *u, const blam_real3d *v)
{
  const blam_real *us = &u->components[0];
  const blam_real *vs = &v->components[0];
  blam_real3d result = 
  {{
    (blam_real)((blam_real_highp)us[1] * (blam_real_highp)vs[2] - (blam_real_highp)us[2] * (blam_real_highp)vs[1]),
    (blam_real)((blam_real_highp)us[2] * (blam_real_highp)vs[0] - (blam_real_highp)us[0] * (blam_real_highp)vs[2]),
    (blam_real)((blam_real_highp)us[0] * (blam_real_highp)vs[1] - (blam_real_highp)us[1] * (blam_real_highp)vs[0])
  }};
  return result;
}

/**
 * \brief Returns the scalar triple product of \a u, \a v, \a w.
 *
 * In infinite precision, this is operation is implemented as if by 
 * `dot(u, cross(v, w))`.
 *
 * I have not encountered this function in Halo, so treat it as an extension.
 */
static inline
blam_real_highp blam_real3d_scalar_triple(
  const blam_real3d *u,
  const blam_real3d *v,
  const blam_real3d *w)
{
  const blam_real *us = &u->components[0];
  const blam_real *vs = &v->components[0];
  const blam_real *ws = &w->components[0];
  
  const blam_real_highp cross[3] = 
  {
    (blam_real_highp)vs[1] * (blam_real_highp)ws[2] - (blam_real_highp)vs[2] * (blam_real_highp)ws[1],
    (blam_real_highp)vs[2] * (blam_real_highp)ws[0] - (blam_real_highp)vs[0] * (blam_real_highp)ws[2],
    (blam_real_highp)vs[0] * (blam_real_highp)ws[1] - (blam_real_highp)vs[1] * (blam_real_highp)ws[0]
  };
  
  blam_real_highp dot = us[0] * cross[0] + us[1] * cross[1] + us[2] * cross[2];
  return dot;
}

/**
 * \brief Returns the determinant of vectors \a u and \a v.
 */
static inline
blam_real_highp blam_real2d_det(const blam_real2d *u, const blam_real2d *v)
{
  return (blam_real_highp)u->components[0] * (blam_real_highp)v->components[1]
    - (blam_real_highp)u->components[1] * (blam_real_highp)v->components[0];
}

/** 
 * \brief Tests point \a v against \a plane.
 *
 * \return The signed distance between \a plane and the point;
 *         a negative value if behind \a plane and in front otherwise.
 */
static inline
blam_real_highp blam_plane3d_test(const blam_plane3d *plane, const blam_real3d *v)
{
  return blam_real3d_dot(&plane->normal, v) - plane->d;
}

/** 
 * \brief Tests point \a v against \a plane.
 *
 * \return The signed distance between \a plane and the point;
 *         a negative value if behind \a plane and in front otherwise.
 */
static inline
blam_real_highp blam_plane2d_test(const blam_plane2d *plane, const blam_real2d *v)
{
  return blam_real2d_dot(&plane->normal, v) - plane->d;
}

enum blam_projection_plane // should be enum_short but its written as dword
{
  k_blam_projection_plane_yz,
  k_blam_projection_plane_xz,
  k_blam_projection_plane_xy
};

/**
 * \brief Returns the cardinal plane onto which space would project onto with 
 *        greatest fidelity for some plane with normal \a v.
 */
static inline
enum blam_projection_plane blam_real3d_projection_plane(const blam_real3d *v)
{
  const blam_real x = fabs(v->components[0]);
  const blam_real y = fabs(v->components[1]);
  const blam_real z = fabs(v->components[2]);

  if (z >= y && z >= x)
    return k_blam_projection_plane_xy;
  else if (y >= x && y >= z)
    return k_blam_projection_plane_xz;
  else
    return k_blam_projection_plane_yz;
}

/**
 * \brief Gets the projective component indices for a (possibly inverted) plane.
 *
 * \param [in] plane            The plane to project onto.
 * \param [in] is_forward_plane Controls the winding order; supply as \c true to get
 *                              the forward order, otherwise \c false for the 
 *                              inverted order.
 */
static inline
blam_pair_int blam_projection_plane_indices(
  enum blam_projection_plane plane,
  blam_bool                  is_forward_plane)
{
  // Halo uses a lookup-table for this operation. See above for the LUT.
  // The following is equivalent. 
  const int primary_index = 2 - is_forward_plane + plane;
  const int secondary_index = 1 + is_forward_plane + plane;
  
  blam_pair_int result = {
    .first = primary_index < 3 ? primary_index : primary_index - 3,
    .second = secondary_index < 3 ? secondary_index : secondary_index - 3
  };
  
  return result;
}

/**
 * \brief Returns the projection of \a v onto a canonical \a plane.
 *
 * \param [in] v                The vector to project.
 * \param [in] plane            The plane to project onto.
 * \param [in] is_forward_plane Controls the winding order; supply as \c true to get
 *                              the forward order, otherwise \c false for the 
 *                              inverted order.
 */
static inline
blam_real2d blam_real3d_projected_components(
  const blam_real3d *restrict v,
  enum blam_projection_plane  plane,
  blam_bool                   is_forward_plane)
{
  blam_pair_int i = blam_projection_plane_indices(plane, is_forward_plane);
  blam_real2d result = {{v->components[i.first], v->components[i.second]}};
  
  return result;
}

#endif // BLAM_MATH_H