api_version = "1.12.0.0"

local ffi = require "ffi"

local bsp_fixes
function OnScriptLoad()
    bsp_fixes = ffi.load("hlef")
end

function OnScriptUnload()
    
end