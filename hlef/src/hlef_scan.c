#include "hlef_scan.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <string.h>
#include <ctype.h>

#include <windows.h>
#include <psapi.h>
#include <dbghelp.h>

#include "hlef_patch.h"

enum hlef_scan_state
{
    k_hlef_scan_state_first_nibble,
    k_hlef_scan_state_second_nibble,
    
    k_hlef_scan_state_terminating,
    k_hlef_scan_state_match = k_hlef_scan_state_terminating,
    k_hlef_scan_state_mismatch,
};

enum hlef_scan_token_type
{
    k_hlef_scan_token_wildcard,
    k_hlef_scan_token_literal,
    k_hlef_scan_token_end
};

struct hlef_scan_token
{
    enum hlef_scan_token_type type;
    union {
        char          c;
        unsigned long n;
    };
};

static
enum hlef_scan_state get_next_scan_state(
    enum hlef_scan_state         state,
    const struct hlef_scan_token token,
    const char** stream,
    const char*  stream_end)
{
    if (token.type == k_hlef_scan_token_end)
        return k_hlef_scan_state_match;
    else if (*stream == stream_end)
        return k_hlef_scan_state_mismatch;
    
    unsigned char i = **stream;
    int advance = 0;
    switch (state) {
    case k_hlef_scan_state_first_nibble:
        if (token.type == k_hlef_scan_token_wildcard) {
            state = k_hlef_scan_state_second_nibble;
        } else {
            state = (i >> 4ul) == token.n
                ? k_hlef_scan_state_second_nibble
                : k_hlef_scan_state_mismatch;
        }
        break;
    case k_hlef_scan_state_second_nibble:
        if (token.type == k_hlef_scan_token_wildcard) {
            state = k_hlef_scan_state_first_nibble;
            advance = 1;
        } else {
            state = (i & 0x0Ful) == token.n
                ? k_hlef_scan_state_first_nibble
                : k_hlef_scan_state_mismatch;
            advance = state != k_hlef_scan_state_mismatch ? 1 : 0;
        }
        break;
    default: break;
    }
    
    *stream += advance;
    return state;
}

static
int hlef_test_byte_signature(
    const char* signature,
    const char* cursor,
    const char* last)
{
    enum hlef_scan_state state = k_hlef_scan_state_first_nibble;
    while (state < k_hlef_scan_state_terminating) {
        for (; *signature && isspace((unsigned char)*signature); ++signature) ;
        
        struct hlef_scan_token token;
        token.c = *signature;
        if (token.c == '?') {
            token.type = k_hlef_scan_token_wildcard;
        } else if (token.c == '\0') {
            token.type = k_hlef_scan_token_end;
        } else if (isxdigit((unsigned char)token.c)) {
            char buf[] = {signature[0], '\0'};
            token.type = k_hlef_scan_token_literal;
            token.n    = strtoul(buf, NULL, 16);
        } else {
            // malformed byte signature
            state = k_hlef_scan_state_mismatch;
            break;
        }
        
        state = get_next_scan_state(state, token, &cursor, last);
        ++signature;
    }
    
    return state == k_hlef_scan_state_match;
}

/**
 * \brief Scans a range for the first match of a byte signature.
 *
 * \return A pointer to the first match, otherwise \c NULL.
 */
static
const char* hlef_scan_byte_signature_in_range(
    const char* sig, 
    const char* first,
    const char* last)
{
    const char* cursor = first;
    
    // make region readable
    DWORD flOldProtect = 0;
    if (!VirtualProtect((LPVOID)first, last - first, PAGE_READONLY, &flOldProtect))
        return NULL;
    
    for (; cursor != last; ++cursor) {
        if (hlef_test_byte_signature(sig, cursor, last))
            break;
    }
    
    // restore protections
    VirtualProtect((LPVOID)first, last - first, flOldProtect, &flOldProtect);
    
    return cursor != last ? cursor : NULL;
}

/**
 * \brief Scans a module for the first match of a byte signature.
 *
 * \return A pointer to the first match, otherwise \c NULL.
 */
static
void* hlef_scan_byte_signature(const char* module, const char* sig)
{
    if (!sig)
        return NULL;
    
    HMODULE hModule = GetModuleHandle(module);
    if (hModule == NULL)
        return NULL;
    
    void* image_base = NULL;
    {
        MODULEINFO mi;
        memset(&mi, 0, sizeof(mi));
        
        if (GetModuleInformation(GetCurrentProcess(), hModule, &mi, sizeof(mi)))
            image_base = (void*)mi.lpBaseOfDll;
    }
    IMAGE_NT_HEADERS* image_header = ImageNtHeader(hModule);
    
    if (image_base == NULL || image_header == NULL)
        return NULL;
    
    
    IMAGE_SECTION_HEADER* image_section_headers = (IMAGE_SECTION_HEADER*)(image_header + 1);
    
    IMAGE_SECTION_HEADER* header = image_section_headers;
    IMAGE_SECTION_HEADER* header_last = image_section_headers + image_header->FileHeader.NumberOfSections;
    for (; header != header_last; ++header) {
        if ((header->Characteristics & IMAGE_SCN_CNT_CODE) != IMAGE_SCN_CNT_CODE)
            continue;
        
        const char* seg_base = (const char*)image_base + header->VirtualAddress;
        const char* seg_last = seg_base + header->Misc.VirtualSize;
        
        const char* search = hlef_scan_byte_signature_in_range(sig, seg_base, seg_last);
        if (search)
            return (void*)search;
    }
    
    return NULL;
}

static
int32_t hlef_read_i32(const char* p_)
{
    unsigned char* p = (unsigned char*)p_;
    uint32_t result = 0;
    
    DWORD flOldProtect = 0;
    VirtualProtect((LPVOID)p, sizeof(uint32_t), PAGE_READONLY, &flOldProtect);
    
    result
        = p[0]
        | (p[1] << CHAR_BIT)
        | (p[2] << (2 * CHAR_BIT))
        | (p[3] << (3 * CHAR_BIT));
    
    VirtualProtect((LPVOID)p, sizeof(uint32_t), flOldProtect, &flOldProtect);
    
    return (int32_t)result;
}

static
uint32_t hlef_read_u32(const char* p)
{
    return (uint32_t)hlef_read_i32(p);
}

static
void hlef_scan_step_move(
    const struct hlef_scan_signature *sig,
    const struct hlef_scan_step *step,
    char** pcursor)
{
    (void)sig;
    assert(kr_hlef_scan_step_move_type <= step->type && step->type < kr_hlef_scan_step_move_type_end);
    
    const ptrdiff_t offset = step->offset;
    char* cursor = *pcursor;
    switch (step->type) {
        case k_hlef_scan_translate:
            cursor += offset;
            break;
        case k_hlef_scan_mem32:
            cursor = (char*)hlef_read_u32(cursor + offset);
            break;
        case k_hlef_scan_rel32:
            cursor = cursor + offset + hlef_read_i32(cursor);
            break;
        default:
            break;
    }
    *pcursor = cursor;
} 

static
void hlef_scan_step_read(
    const struct hlef_scan_signature *sig,
    const struct hlef_scan_step *step,
    char** pcursor)
{
    assert(kr_hlef_scan_step_read_type <= step->type && step->type < kr_hlef_scan_step_read_type_end);
    
    void** dst = step->dst ? (void**)step->dst : sig->destination;
    assert(dst);
    
    const ptrdiff_t offset = step->offset;
    char* cursor = *pcursor;
    switch (step->type) {
        case k_hlef_scan_read_cursor:
            *dst = cursor + offset;
            break;
        case k_hlef_scan_read_rel32:
            *dst = cursor + hlef_read_i32(cursor) + offset;
            break;
        case k_hlef_scan_read_memory:
        {
            DWORD flOldProtect;
            VirtualProtect(
                cursor + offset, step->len,
                PAGE_READONLY,
                &flOldProtect);
            memcpy(dst, cursor + offset, step->len);
            VirtualProtect(
                cursor + offset, step->len,
                flOldProtect,
                &flOldProtect);
        }   break;
        default:
            break;
    }
    *pcursor = cursor;
} 

static
void* write_mem_address(
    const struct hlef_scan_signature *sig,
    const struct hlef_scan_step *step,
    char* cursor)
{
    (void)sig;
    switch (step->type) {
        case k_hlef_scan_write_rel32:
            return cursor;
        case k_hlef_scan_write_nop:
        case k_hlef_scan_write_memory:
            return cursor + step->offset;
        default:
            return NULL;
    }
}

static
size_t write_mem_size(
    const struct hlef_scan_signature *sig,
    const struct hlef_scan_step *step,
    char* cursor)
{
    (void)sig;
    (void)cursor;
    switch (step->type) {
        case k_hlef_scan_write_nop:
            return step->len;
        case k_hlef_scan_write_rel32:
            return sizeof(int32_t);
        case k_hlef_scan_write_memory:
            return step->len ? step->len : strlen((const char*)step->src);
        default:
            return 0;
    }
}

static
void hlef_scan_step_write(
    const struct hlef_scan_signature *sig,
    const struct hlef_scan_step *step,
    char** pcursor)
{
    assert(kr_hlef_scan_step_write_type <= step->type && step->type < kr_hlef_scan_step_write_type_end);
    
    char* cursor = *pcursor;
    
    void*  write_address = write_mem_address(sig, step, cursor);
    size_t write_len     = write_mem_size(sig, step, cursor);
    
    assert(write_address != NULL);
    assert(write_len > 0);
    
    DWORD flOldProtect;
    VirtualProtect(
        write_address, write_len, 
        PAGE_READWRITE, 
        &flOldProtect);
    
    hlef_patches_add(write_address, write_len);
    switch (step->type) {
        case k_hlef_scan_write_nop:
            memset(write_address, 0x90, write_len);
            break;
        case k_hlef_scan_write_rel32:
        {
            assert(step->dst);
            const int32_t rel32 = (int32_t)((char*)step->dst - cursor - step->offset);
            memcpy(write_address, &rel32, write_len);
        }   break;
        case k_hlef_scan_write_memory:
            assert(step->src);
            memcpy(write_address, step->src, write_len);
            break;
        default:
            assert(false);
            break;
    }
    
    VirtualProtect(
        write_address, write_len, 
        flOldProtect, 
        &flOldProtect);
    
    *pcursor = cursor;
} 

void* hlef_scan(const struct hlef_scan_signature *sig)
{
    char* cursor = hlef_scan_byte_signature(sig->module, sig->byte_signature);
    
    if (cursor) {
        const struct hlef_scan_step *step = sig->steps;
        for (; step->type != k_hlef_scan_return; ++step) {
            switch (step->type) {
                case k_hlef_scan_translate:
                case k_hlef_scan_mem32:
                case k_hlef_scan_rel32:
                    hlef_scan_step_move(sig, step, &cursor);
                    break;
                case k_hlef_scan_read_cursor:
                case k_hlef_scan_read_rel32:
                case k_hlef_scan_read_memory:
                    hlef_scan_step_read(sig, step, &cursor);
                    break;
                case k_hlef_scan_write_nop:
                case k_hlef_scan_write_rel32:
                case k_hlef_scan_write_memory:
                    hlef_scan_step_write(sig, step, &cursor);
                    break;
                default: break;
            }
        }
    }
    
    if (sig->destination && *(sig->destination) == NULL)
        *(sig->destination) = cursor;
    return cursor;
}