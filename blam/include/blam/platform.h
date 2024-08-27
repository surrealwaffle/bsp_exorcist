#ifndef BLAM_PLATFORM_H
#define BLAM_PLATFORM_H

#if defined(__GNUC__)
# define BLAM_ATTRIBUTE(...) __attribute__ ((__VA_ARGS__))
# define BLAM_ASSUME(cond) if (!(cond)) __builtin_unreachable()
# define BLAM_EXPECT(exp, c) __builtin_expect ((exp), (c))
#else
# define BLAM_ATTRIBUTE(...)
# define BLAM_ASSUME(cond) 
# define BLAM_EXPECT(exp, c) (exp)
#endif

#define BLAM_LIKELY(exp)   BLAM_EXPECT(!!(exp), 1)
#define BLAM_UNLIKELY(exp) BLAM_EXPECT(!!(exp), 0)

#endif // BLAM_PLATFORM_H