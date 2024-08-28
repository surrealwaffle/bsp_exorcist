# bsp_exorcist
Phantom BSP mitigations for Halo PC (Retail and Custom Edition) servers and clients.

## Technique
`bsp_exorcist` provides a replacement subroutine for `collision_bsp_test_vector` 
that provides mitigations for phantom BSP. The mitigation consists of a quick test 
which admits most real surfaces, followed by a second, more expensive test which 
admits real surfaces and rejects most forms of phantom BSP. This technique works 
well under the assumption that phantom BSP and BSP holes are rare.

## Why does phantom BSP occur?
`tool.exe` may compile incorrect BSPs (both for map and collision models), 
especially when nearly coplanar surface warnings are emitted. This behaviour 
produces both BSP leaks as well as phantom BSP. It must be stressed that Halo's BSP 
traversal subroutines are implemented correctly. The root cause of phantom BSP is 
incorrect splitting planes.

The leaves of a 3D BSP tree form disjoint, convex volumes defined by splitting 
planes of each leaf. The splitting planes either divide the leaf from other leaves 
or interior volume (within the BSP) from exterior volume (outside the BSP). The 
planes which divide interior from exterior have associated surface data. In the 
presence of double-sided surfaces, planes which divide two interior volumes may also
have surface data.

During BSP-vector intersection tests at a leaf that does not contain double-sided 
surfaces, Halo finds the BSP2D reference within a leaf associated with the plane 
that divides the leaf from BSP exterior through the vector. In this situation, Halo 
assumes that the BSP2D located spans the extents of the dividing plane. This 
assumption is based on the sealed world property. 

However, if the leaf was not split correctly, then phantom BSP occurs when that 
assumption does not hold, resulting in excess surface area attributed to the located
surface. This results in the well-known phantom BSP on Danger Canyon.

If we then perform a Surface3D-vector intersection test to verify that a surface was 
indeed intersected, holes are then punched into the BSP (different from BSP leaks) 
even in infinite precision, because the dividing planes are incorrect and may cut 
into valid surface in another leaf. This is why con's [ghostbuster](https://opencarnage.net/index.php?/topic/8069-ghostbuster-a-phantom-bsp-tag-fixer-deprecated/)
tool produces holes. The most obvious example of this is a face of the central 
pillar on Wizard, as shown below.

![wizard_phantom_bsp](/wizard_phantom_bsp.png)

The red arrow into the face indicates where the vector intersects with the surface.
The red outline indicates the surface intersected. The lime-green line indicates 
where the top and bottom surfaces are split along the face. As shown, excess 
geometry is attributed to the bottom surface, and eliminating this geometry would 
introduce a hole into the BSP.

## TODOs
 * MSVC build support
 * Reverse `collision_debug_phantom_bsp` to determine how Sapien detects phantom BSP
 * Convert to C++ (maybe)
 * Mitigate BSP leaks