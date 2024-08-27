--[[
------------------------------------------------------------------------------
 "THE DONUT-WARE LICENSE" (Revision 0x1EA7F00D):
 Pirate wrote this file. As long as you retain this notice you can do whatever 
 you want with this stuff. If we meet some day, and you think this stuff is 
 worth it, you can buy me a donut in return.
------------------------------------------------------------------------------
--]]

-- This file provides a simple interface for the functions provided by the 
-- C library hlef, which you can find as part of hlm at this repository:
-- https://github.com/surrealwaffle/sapp-hlm

local ffi = require "ffi"

ffi.cdef[[

typedef int32_t hlef_index_long;
typedef int16_t hlef_index_word;
typedef int8_t  hlef_index_byte;

typedef uint32_t hlef_identity;
typedef float    hlef_real;

struct hlef_real3d
{
    hlef_real components[3];
};

struct hlef_leaf_cluster_location
{
    hlef_index_long leaf;
    hlef_index_word cluster;
};

// See blam/include/blam/collision.h for documentation on these options
struct blam_collision_test_ext_params {
    struct {
        bool mitigate;
    } phantom_bsp;
    
    struct {
        bool mitigate;

        float dot_product_threshold;
        float square_distance_threshold;
    } bsp_hole;
}; 

void hlef_object_disconnect(hlef_identity object_id);

void hlef_object_connect(
    hlef_identity object_id,
    struct hlef_leaf_cluster_location *hint);

void hlef_object_recalculate_nodes(hlef_identity object_id);

uint16_t hlef_random_next(
    int advance,
    int pseudorandom);

int16_t hlef_random_short(
    int16_t min, int16_t max,
    int advance,
    int pseudorandom);
    
hlef_real hlef_random_real(
    hlef_real min, hlef_real max,
    int advance,
    int pseudorandom);

void hlef_get_collision_test_ext_params(
    struct blam_collision_test_ext_params *params);

void hlef_set_collision_test_ext_params(
    const struct blam_collision_test_ext_params *params);

]]

local chlef = ffi.load("hlef")

local hlef = {
    object_disconnect = chlef.hlef_object_disconnect,
    object_connect = chlef.hlef_object_connect,
    object_recalculate_nodes = chlef.hlef_object_recalculate_nodes,
    random_next = function(opt)
        opt = opt or {}
        local advance = opt.advance or true
        local pseudorandom = opt.pseudorandom or true
        return chlef.hlef_random_next(advance, pseudorandom)
    end,
    random_short = function(imin, imax, opt)
        opt = opt or {}
        local advance = opt.advance or true
        local pseudorandom = opt.pseudorandom or true
        return chlef.hlef_random_short(imin, imax, advance, pseudorandom)
    end,
    random_real = function(vmin, vmax, opt)
        opt = opt or {}
        local advance = opt.advance or true
        local pseudorandom = opt.pseudorandom or true
        return chlef.hlef_random_next(vmin, vmax, advance, pseudorandom)
    end,
    get_collision_test_ext_params = function()
        local result = ffi.new("struct blam_collision_test_ext_params[1]", {})
        chlef.hlef_get_collision_test_ext_params(result)
        return result[0]
    end,
    set_collision_test_ext_params = function(params)
        local ext = ffi.new("struct blam_collision_test_ext_params[1]", 
            {[0]=params})
        chlef.hlef_set_collision_test_ext_params(ext)
    end
}

return hlef