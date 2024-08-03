# bsp_exorcist
Phantom BSP mitigations for Halo PC (Retail and Custom Edition) servers and clients.

## Technique
`bsp_exorcist` provides a ray-BSP intersection subroutine that performs an 
inexpensive test that admits phantom BSP surfaces (with false positives) and then 
applies an additional, more involved test which rejects those false positives at the
cost of rejecting some true positives. 

Under the assumption that phantom BSP and BSP holes are rare, this technique works 
well.

## TODOs
 * MSVC build support
 * Reverse `collision_debug_phantom_bsp` to determine how Sapien detects phantom BSP
 * Convert to C++ (maybe)