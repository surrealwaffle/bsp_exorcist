#ifndef BLAM_BASE_H
#define BLAM_BASE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <assert.h>
#include <limits.h>

#include "platform.h"

#if !defined(NDEBUG) && __STDC_VERSION__ >= 201112L
# include <assert.h>
# define BLAM_ASSERT_SIZE(type, cbBytes) _Static_assert(sizeof(type) == cbBytes, "sizeof(" #type ") constraint")
#else
# define BLAM_ASSERT_SIZE(type, cbBytes)
#endif

typedef int32_t  blam_long;
typedef int16_t  blam_short;
typedef int8_t   blam_byte;
typedef uint32_t blam_ulong;
typedef uint16_t blam_ushort;
typedef uint8_t  blam_ubyte;

#if BOOL_TYPE_SIZE == 1
typedef _Bool     blam_bool;
#else
typedef blam_byte blam_bool;
#endif

typedef blam_long   blam_index_long;
typedef blam_short  blam_index_short;
typedef blam_byte   blam_index_byte;

typedef blam_ulong  blam_flags_long;
typedef blam_ushort blam_flags_short;
typedef blam_ubyte  blam_flags_byte;

typedef blam_long   blam_enum_long;
typedef blam_short  blam_enum_short;
typedef blam_byte   blam_enum_byte;

typedef float       blam_real;

typedef uint32_t    blam_datum_index;

typedef struct blam_real3d
{
  blam_real components[3];
} blam_real3d; BLAM_ASSERT_SIZE(blam_real3d, 0x0C);

typedef struct blam_real2d
{
  blam_real components[2];
} blam_real2d; BLAM_ASSERT_SIZE(blam_real2d, 0x08);

typedef struct blam_plane3d
{
  blam_real3d normal;
  blam_real   d;
} blam_plane3d; BLAM_ASSERT_SIZE(blam_plane3d, 0x10);

typedef struct blam_plane2d
{
  blam_real2d normal;
  blam_real   d;
} blam_plane2d; BLAM_ASSERT_SIZE(blam_plane2d, 0x0C);

typedef struct blam_pair_byte
{
  blam_byte first;
  blam_byte second;
} blam_pair_byte;

typedef struct blam_pair_short
{
  blam_short first;
  blam_short second;
} blam_pair_short;

typedef struct blam_pair_long
{
  blam_long first;
  blam_long second;
} blam_pair_long;

typedef struct blam_pair_int
{
  int first;
  int second;
} blam_pair_int;

typedef struct blam_pair_ubyte
{
  blam_ubyte first;
  blam_ubyte second;
} blam_pair_ubyte;

typedef struct blam_pair_ushort
{
  blam_ushort first;
  blam_ushort second;
} blam_pair_ushort;

typedef struct blam_pair_ulong
{
  blam_ulong first;
  blam_ulong second;
} blam_pair_ulong;

struct blam_structure_location
{
  blam_index_long  leaf;
  blam_index_short cluster;
}; BLAM_ASSERT_SIZE(struct blam_structure_location, 0x08);

struct blam_bit_vector
{
  blam_short count;  ///< The number of bits available at #state.
  blam_ulong *state; ///< The buffer of bits.
}; BLAM_ASSERT_SIZE(struct blam_bit_vector, 0x08);

/**
 * \brief Tests the state of a single bit.
 *
 * If `bit < 0` or `bit >= bvec->count`, the result is undefined.
 *
 * \param[in] bvec The bit vector.
 * \param[in] bit The index of the bit to test.
 * \return \c true if the bit is set, otherwise \c false.
 */
static inline
blam_bool blam_bit_vector_test(const struct blam_bit_vector *bvec, int bit)
{
  assert(bvec && bvec->state && bvec->count > 0);
  assert(0 <= bit && bit < bvec->count);
  
  const int index = bit / (CHAR_BIT * sizeof(*bvec->state));
  bit %= CHAR_BIT * sizeof(*bvec->state);
  return !!(bvec->state[index] & ((blam_ulong)1 << bit));
}

// Unsafe sanitize (does not check if integer is -1)
static inline 
blam_long  blam_sanitize_long(blam_long l)   { return (blam_long)(l & 0x7FFFFFFFuL); }

static inline 
blam_short blam_sanitize_short(blam_short s) { return (blam_short)(s & 0x7FFFu); }

static inline 
blam_byte  blam_sanitize_byte(blam_byte b)   { return (blam_byte)(b & 0x7Fu); }

// Safe sanitize (returns -1 if integer is -1)
static inline 
blam_long  blam_sanitize_long_s(blam_long l)   { return l != -1 ? blam_sanitize_long(l) : l; }

static inline 
blam_short blam_sanitize_short_s(blam_short s) { return s != -1 ? blam_sanitize_short(s) : s; }

static inline 
blam_byte  blam_sanitize_byte_s(blam_byte b)   { return b != -1 ? blam_sanitize_byte(b) : b; }

/**
 * \brief Returns the array index part of a datum index.
 */
static inline 
int blam_index(blam_datum_index index)
{
  return blam_sanitize_short((blam_short)index); 
}

/**
 * \brief Returns the ID part of a datum index.
 */
static inline 
blam_ushort blam_identifier(blam_datum_index index)
{
  return (blam_ushort)(index >> (CHAR_BIT * sizeof(blam_short))); 
}

#endif // BLAM_BASE_H