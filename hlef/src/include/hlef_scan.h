#ifndef HLEF_SCAN_H
#define HLEF_SCAN_H

#include <stddef.h>

#define HLEF_SCAN_STEPS(...)    \
    {                           \
        __VA_ARGS__,            \
        {k_hlef_scan_return}    \
    }

enum hlef_scan_step_type
{
    kr_hlef_scan_step_move_type,
    k_hlef_scan_translate  ///< Moves the cursor by \c offset.
        = kr_hlef_scan_step_move_type, 
    k_hlef_scan_mem32,     ///< Moves the cursor into a pointer, at 
                           ///< \c offset from the cursor.
    k_hlef_scan_rel32,     ///< Moves the cursor by 32-bit displacement at the 
                           ///< cursor, set to `cursor + offset + displacement`.
    kr_hlef_scan_step_move_type_end,
    
    kr_hlef_scan_step_read_type,
    k_hlef_scan_read_cursor ///< Sets `*dst` to `cursor + offset`.
        = kr_hlef_scan_step_read_type,
    k_hlef_scan_read_rel32,  ///< Sets `*dst` to `cursor + dword[cursor] + offset`.
    k_hlef_scan_read_memory, ///< Copies \c len bytes starting from 
                             ///< `cursor + offset` to \c dst.
    kr_hlef_scan_step_read_type_end,
    
    kr_hlef_scan_step_write_type,
    k_hlef_scan_write_nop    ///< Writes `len` no-op instructions starting at
                             ///< `cursor + offset`.
        = kr_hlef_scan_step_write_type,
                             ///< A restore point is automatically generated.
    k_hlef_scan_write_rel32, ///< Writes `dst - cursor - offset` as a 32-bit
                             ///< integer to the memory at the cursor.
                             ///< A restore point is automatically generated.
    k_hlef_scan_write_memory,///< Writes \c len bytes from \c src to memory 
                             ///< starting at `cursor + offset`, or \c src as a
                             ///< null-terminated string if \c len is \c 0.
                             ///< A restore point is automatically generated.
    kr_hlef_scan_step_write_type_end,
    
    k_hlef_scan_return ///< Stops the scan and returns \c cursor.
};

struct hlef_scan_step
{
    enum hlef_scan_step_type type;
    ptrdiff_t                offset;
    
    union {
        void*  dst;
        void*  src;
    };
    
    union {
        size_t len;
    };
};

struct hlef_scan_signature
{
    void**      destination; ///< If not \c NULL, receives the final cursor.
    const char* module;
    const char* byte_signature;
    struct hlef_scan_step steps[]; // VLA
};

void* hlef_scan(const struct hlef_scan_signature* sig);

#endif // HLEF_SCAN_H