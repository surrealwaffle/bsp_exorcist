# bsp_exorcist
Phantom BSP mitigations for Halo PC (Retail and Custom Edition) servers and clients.

## Technique
`bsp_exorcist` provides a replacement subroutine for `collision_bsp_test_vector` 
that provides mitigations for phantom BSP. The mitigation consists of a quick test 
which admits most real surfaces, followed by a second, more expensive test which 
admits real surfaces and rejects most forms of phantom BSP. This technique works 
well under the assumption that phantom BSP and BSP holes are rare.

Note: In-engine, Halo refers to rays in this context as vectors. To keep things 
consistent, I'll use Halo's terminology here.

## Build
To build this project, a compiler that supports `C11` is required and involves the 
usual `CMake` build process.

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cd build
cmake --build .
```

This builds `hlef.dll`, which applies the necessary patches upon being loaded in an 
all-or-nothing fashion. The patches *should* work for both server and client 
binaries.

Users will need some way to load this library. This exercise is left to the reader.

## Why does phantom BSP occur?
`tool.exe` may compile incorrect BSPs (both for map and collision models), 
especially when nearly coplanar surface warnings are emitted. This behaviour 
produces both BSP leaks as well as phantom BSP. It must be stressed that Halo's BSP 
traversal subroutines are implemented correctly. The root cause of phantom BSP is 
incorrect splitting planes.

The leaves of a 3D BSP tree form disjoint, convex volumes defined by the splitting 
planes of each leaf. The splitting planes either divide leaves, OR interior volume 
(within the BSP) from exterior volume (outside the BSP). The planes which divide 
interior from exterior have associated surface data. In the presence of double-sided 
surfaces, planes which divide two interior volumes may also have surface data.

During BSP-vector intersection tests at a leaf, Halo finds the BSP2D reference 
within a leaf associated with the last splitting plane intersected. Ignoring 
double-sided surfaces, Halo assumes that if such a BSP2D is found, it must 
span the extents of the splitting plane within that leaf. However, if the leaves of 
the BSP are not correctly split, then phantom BSP occurs where that assumption does 
not hold, resulting in excess area being attributed to surfaces.

In the case of the well-known Danger Canyon ramp phantom BSP, excess surface area is 
generated where there is no surface. Such instances of phantom BSP can be 
corrected by testing the vector against the surface to verify that the vector 
actually intersects the surface. 

However, consider also the possibility of phantom BSP that occurs over other 
surfaces. Performing the surface-vector intersection test in such instances will 
create holes in the BSP (different from leaks) because an incorrectly placed 
splitting plane may cut a valid surface in another leaf. This is why con's [ghostbuster](https://opencarnage.net/index.php?/topic/8069-ghostbuster-a-phantom-bsp-tag-fixer-deprecated/)
technique produces holes. Consider an example from Wizard.

![wizard_phantom_bsp](/wizard_phantom_bsp.png)

 * The red arrow into the face indicates where the vector intersects with the surface.
 * The red outline indicates the surface intersected. 
 * The lime-green line indicates the top-bottom leaf split in the BSP. 

As shown, excess geometry is attributed to the bottom surface. Eliminating that 
excess geometry would introduce a hole into the BSP, because the surface that 
should be intersected is never tested in the first place.

## TODOs
 * MSVC build support
 * Reverse `collision_debug_phantom_bsp` to determine how Sapien detects phantom BSP
 * Convert to C++ (maybe)
 * Mitigate BSP leaks
 * Provide APIs to control mitigation behaviour