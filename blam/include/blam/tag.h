#ifndef BLAM_TAG_H
#define BLAM_TAG_H

#include "base.h"

#define BLAM_TAG_BLOCK_BASE(tag, var, block) (__typeof__(var))(tag)->block.address
#define BLAM_TAG_BLOCK_GET(tag, var, block, idx) (BLAM_TAG_BLOCK_BASE(tag, var, block) + (idx))

struct blam_tag_block
{
  blam_long count;
  void*     address;
  void*     definition; // unused unless in dev tools
}; BLAM_ASSERT_SIZE(struct blam_tag_block, 0x0C);

#endif // BLAM_TAG_H