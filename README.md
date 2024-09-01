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

# Writeup
This section will document how Halo represents and traverses BSP structures, and why
phantom BSP and BSP leaks occur.

If you are more comfortable looking at code, see the branch named 
`baseline-no-fixes`, which contains the BSP-vector intersection subroutine as 
implemented in Halo PC. Note that in this version of the code, I refer to BSP leaks 
as holes. I later settled on the term BSP leak to differentiate it from holes that 
may be introduced through mitigation methods.

## Background Knowledge
This section assumes a bit of background knowledge in BSP trees and related 
mathematical theory. To explain everything with the rigour desired would require 
much more time and space than a small readme could offer.

Instead, for those who don't know what a BSP tree is or does, the short of it is 
that a BSP recursively subdivides space through partitioning hyperplanes (space 
above/below a line in 2D, a plane in 3D) and has properties that make it useful 
for certain operations like point-inclusion tests and directed line segment 
intersections. 

For Halo specifically, 3D BSPs are organized such that leaves contain references to 
2D BSPs that map onto the partitioning planes which contain solid surface data for 
that leaf. These references include the ID of the plane, so that when a partitioning 
plane is intersected in a leaf, the 2D BSP associated with that plane can be quickly 
located. Halo optimizes this process by employing the sealed world property so 
that - ignoring double-sided surfaces - intersections are only tested across planes 
that partition the BSP interior (in bounds) from exterior (out of bounds).

## Why does phantom BSP occur?
In the above test, if a 2D BSP is found, the span of its partioning plane within 
that leaf is assumed to be solid. When such a partitioning plane partially spans 
empty space, excess surface area is attributed to the surface determined through 
the 2D BSP. This is why the well-known Danger Canyon ramp phantom BSP occurs.

It should be emphasized that Halo implements the traversal subroutines correctly. 
The cause of phantom BSP is an incorrectly constructed tree, so the fault lies with 
`tool.exe` for not creating the correct partitions and on the level artist for 
ignoring nearly-coplanar surface warnings.

One might propose to verify that surface is present at the point of intersection 
with the plane by intersecting with the surface directly. This is the option that 
that [con's ghostbuster](https://opencarnage.net/index.php?/topic/8069-ghostbuster-a-phantom-bsp-tag-fixer-deprecated/)
took. However, even in infinte precision, this method punches holes (different from 
BSP leaks) into the map because *phantom BSP can occur over other surfaces*. 
Consider the following example.

![wizard_phantom_bsp](/wizard_phantom_bsp.png)

 * The red arrow into the face indicates where the vector intersects with the surface.
 * The red outline indicates the surface intersected. 
 * The lime-green line indicates the top-bottom leaf split in the BSP. 

As shown, excess geometry is attributed to the bottom surface, outside of the 
surface bounds.It may be that this is a result of an intentional optimization 
employed by `tool.exe`, but incorrectly implemented. However, this view is difficult 
to embrace, as some systems such as decal placement check for valid surface.

## Why do BSP leaks occur?
Consider the case that a leaf contains no 2D BSP reference associated with a plane 
that partitions BSP interior from exterior. Halo will not find an intersection in 
this case, and thus the BSP contains a leak. This issue is a bit more troubling to 
mitigate, because the leaf may not even reference a 'close enough' 2D BSP to fall 
back on. Consider the giant leak on Derelict.

![hole larger than sagittarius A*](/carousel_bsp_leak.png)
 
 * The area outlined in red is surface #138, plane #45.
 * The area marked in orange is leaf #648, and features a BSP leak.
 * The area marked in green is leaf #635, and features no BSP leak. It should be noted that this leaf is EXTREMELY slim.

Suppose a line is intersecting `surface #138` through `leaf #635` in the 
green-highlighted area. The partitioning plane crossed is `plane #45`. The 
subroutine will find a BSP2D reference associated with `plane #45` in this leaf and 
correctly determine that the surface intersected is `surface #138`.

Now suppose a line is intersecting `surface #138` through `leaf #648` in the 
orange-highlighted area. The partitioning plane crossed is `plane #45`. The 
subroutine will find no BSP2D reference in this leaf associated with `plane #45`, 
and thus no intersection takes place. Distressingly, there is no BSP2D referenced 
in this leaf whose associated plane is even loosely coplanar with `plane #45`.

BSP leaks are easily detected, but difficult to mitigate due to the expense of 
needing to search for all the BSP2D references associated with a plane of interest. 
One could do some precomputation for a relevant multimap structure, but implementing
it in this way would require a mechanism to recalculate these structure on map load.

## TODOs
 * MSVC build support
 * Reverse `collision_debug_phantom_bsp` to determine how Sapien detects phantom BSP
 * Convert to C++ (maybe)
 * Mitigate BSP leaks
 * Provide APIs to control mitigation behaviour