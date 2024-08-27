#ifndef BLAM_MATH_H
#define BLAM_MATH_H

#include "platform.h"
#include "base.h"

// In general, if restrict would be needed in certain cases, a function would
// instead simply return by value. This generates assembly close to what Halo 
// has anyway.

/**
 * \brief Returns the scalar product between \a u and \a v.
 */
blam_real blam_real3d_dot(const blam_real3d *u, const blam_real3d *v) BLAM_ATTRIBUTE(pure);
  
/**
 * \brief Returns the scalar product between \a u and \a v.
 */
blam_real blam_real2d_dot(const blam_real2d *u, const blam_real2d *v) BLAM_ATTRIBUTE(pure);

/**
 * \brief Returns the magnitude of \a v.
 */
blam_real blam_real3d_norm(const blam_real3d *v) BLAM_ATTRIBUTE(pure);

/**
 * \brief Returns the magnitude of \a v.
 */
blam_real blam_real2d_norm(const blam_real2d *v) BLAM_ATTRIBUTE(pure);

/**
 * \brief Normalizes vector \a v.
 *
 * \param [in,out] v The vector to normalize.
 *
 * \return The magnitude of \a v before normalization.
 */
blam_real blam_real3d_normalize(blam_real3d *v);

/**
 * \brief Normalizes vector \a v.
 *
 * \param [in,out] v The vector to normalize.
 *
 * \return The magnitude of \a v before normalization.
 */
blam_real blam_real2d_normalize(blam_real2d *v);

/**
 * \brief Scales a vector \a v by scalar \a s.
 */
void blam_real3d_scale(blam_real s, blam_real3d *v);

/**
 * \brief Scales a vector \a v by scalar \a s.
 */
void blam_real2d_scale(blam_real s, blam_real2d *v);

/**
 * \brief Returns the sum of vectors \a u and \a v.
 */
blam_real3d blam_real3d_add(const blam_real3d *u, const blam_real3d *v) BLAM_ATTRIBUTE(pure);

/**
 * \brief Returns the sum of vectors \a u and \a v.
 */
blam_real2d blam_real2d_add(const blam_real2d *u, const blam_real2d *v) BLAM_ATTRIBUTE(pure);

/**
 * \brief Returns the difference between vectors \a u and \a v.
 */
blam_real3d blam_real3d_sub(const blam_real3d *u, const blam_real3d *v) BLAM_ATTRIBUTE(pure);

/**
 * \brief Returns the difference between vectors \a u and \a v.
 */
blam_real2d blam_real2d_sub(const blam_real2d *u, const blam_real2d *v) BLAM_ATTRIBUTE(pure);

/**
 * \brief Returns the vector product between \a u and \a v.
 */
blam_real3d blam_real3d_cross(const blam_real3d *u, const blam_real3d *v) BLAM_ATTRIBUTE(pure);

blam_real blam_real2d_det(const blam_real2d *u, const blam_real2d *v) BLAM_ATTRIBUTE(pure);

/** 
 * \brief Tests point \a v against \a plane.
 *
 * \return The signed distance between \a plane and the point;
 *         a negative value if behind \a plane and in front otherwise.
 */
blam_real blam_plane3d_test(const blam_plane3d *plane, const blam_real3d *v) BLAM_ATTRIBUTE(pure);

/** 
 * \brief Tests point \a v against \a plane.
 *
 * \return The signed distance between \a plane and the point;
 *         a negative value if behind \a plane and in front otherwise.
 */
blam_real blam_plane2d_test(const blam_plane2d *plane, const blam_real2d *v) BLAM_ATTRIBUTE(pure);

/** 
 * \brief Tests a point in implicit form against \a plane.
 *
 * In infinite precision, the point tested is given by `origin + scale * delta`.
 *
 * This does not produce values exactly equivalent to calculating the point 
 * and then running it through `blam_plane3d_test`.
 *
 * \return The signed distance between \a plane and the point;
 *         a negative value if behind \a plane and in front otherwise.
 */
blam_real blam_plane3d_test_implicit(
  const blam_plane3d *plane,
  const blam_real3d  *origin,
  const blam_real3d  *delta,
  blam_real           scale) BLAM_ATTRIBUTE(pure);

/** 
 * \brief Tests a point in implicit form against \a plane.
 *
 * In infinite precision, the point tested is given by `origin + scale * delta`.
 *
 * This does not produce values exactly-equivalent to calculating the point 
 * and then running it through `blam_plane2d_test`.
 *
 * \return The signed distance between \a plane and the point;
 *         a negative value if behind \a plane and in front otherwise.
 */
blam_real blam_plane2d_test_implicit(
  const blam_plane2d *plane,
  const blam_real2d  *origin,
  blam_real           scale,
  const blam_real2d  *delta) BLAM_ATTRIBUTE(pure);

/**
 * \brief Returns the projection of \a v onto \a plane.
 */
blam_real3d blam_plane3d_project(const blam_plane3d *plane, const blam_real3d *v) BLAM_ATTRIBUTE(pure);

/**
 * \brief Returns the projection of \a v onto \a plane.
 */
blam_real2d blam_plane2d_project(const blam_plane2d *plane, const blam_real2d *v) BLAM_ATTRIBUTE(pure);
    
blam_long blam_maxl(blam_long a, blam_long b) BLAM_ATTRIBUTE(const);
blam_short blam_maxs(blam_short a, blam_short b) BLAM_ATTRIBUTE(const);
blam_byte blam_maxb(blam_byte a, blam_byte b) BLAM_ATTRIBUTE(const);
blam_real blam_maxf(blam_real a, blam_real b) BLAM_ATTRIBUTE(const);

blam_long blam_minl(blam_long a, blam_long b) BLAM_ATTRIBUTE(const);
blam_short blam_mins(blam_short a, blam_short b) BLAM_ATTRIBUTE(const);
blam_byte blam_minb(blam_byte a, blam_byte b) BLAM_ATTRIBUTE(const);
blam_real blam_maxf(blam_real a, blam_real b) BLAM_ATTRIBUTE(const);

blam_real blam_clampf(blam_real v, blam_real a, blam_real b) BLAM_ATTRIBUTE(const);

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
enum blam_projection_plane blam_real3d_projection_plane(const blam_real3d *v);

/**
 * \brief Gets the projective component indices for a (possibly inverted) plane.
 *
 * \param [in] plane            The plane to project onto.
 * \param [in] is_forward_plane Controls the winding order; supply as \c true to get
 *                              the forward order, otherwise \c false for the 
 *                              inverted order.
 */
blam_pair_int blam_projection_plane_indices(
  enum blam_projection_plane plane,
  blam_bool                  is_forward_plane);

/**
 * \brief Returns the projection of \a v onto a canonical \a plane.
 *
 * \param [in] v                The vector to project.
 * \param [in] plane            The plane to project onto.
 * \param [in] is_forward_plane Controls the winding order; supply as \c true to get
 *                              the forward order, otherwise \c false for the 
 *                              inverted order.
 */
blam_real2d blam_real3d_projected_components(
  const blam_real3d *restrict v,
  enum blam_projection_plane  plane,
  blam_bool                   is_forward_plane);

#endif // BLAM_MATH_H