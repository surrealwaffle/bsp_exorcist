# bsp_exorcist
Runtime phantom BSP and BSP leak patches for Halo PC (Retail and Custom Edition) 
servers and clients.

## Technique
`bsp_exorcist` provides a replacement subroutine for `collision_bsp_test_vector` 
that provides mitigations for phantom BSP. The mitigation consists of a quick test 
which admits most real surfaces, followed by a second, more expensive test which 
admits real surfaces and rejects most forms of phantom BSP. This technique works 
well under the assumption that phantom BSP and BSP leaks are rare.

Note: In-engine, Halo refers to rays in this context as vectors. To keep things 
consistent, I'll use Halo's terminology here.

In greater detail, `bsp_exorcist` follows the usual subroutine for finding a solid 
surface in a leaf through a plane that partitions BSP interior from exterior 
(`collision_bsp_search_leaf` in `blam/src/collision_bsp.c`). If such a surface is 
found, it is validated by performing a vector-surface intersection test using the 
scalar triple product. If this test indicates an intersection, the surface is 
admitted. Otherwise, the vector does not intersect the surface, but outright 
rejecting the surface can lead to some holes (see images in the writeup below). The 
surface is only rejected when the next solid partition features a leak. For 
backfacing potential phantom BSP, the order is reversed; the surface is rejected 
when the previous solid partition features a leak.

To mitigate BSP leaks, the process is a bit more involved because the ecosystem of 
BSP leaks is quite diverse. For the easily detected variants, the mitigation 
consists of searching the leaf for a viable candidate. Should this fail, adjacent 
leaves may be searched using a heuristic for a viable candidate. If this fails, the 
conservative approach is taken and the leak is allowed to occur. 

## Build
To build this project, a compiler that supports `C11` is required and involves the 
usual `CMake` build process.

```
cmake -B build -DCMAKE_BUILD_TYPE=Release
cd build
cmake --build .
```

The compiler must target an `x86` architecture. If your compiler does not do this 
by default, then additional steps on your part will need to be taken.

This builds `hlef.dll`, which applies the necessary patches upon being loaded in an 
all-or-nothing fashion. The patches *should* work for both server and client 
binaries. 

This project currently does not support MSVC. If you manage to build `bsp_exorcist` 
with MSVC and the patches do not introduce instability, please submit a pull 
request. In the meantime, you can grab a release build.

## Installation
Users will need some way to load this library. There are a few ways this can be 
done, depending on your circumstances:
 * for `chimera` users, create a `mods` folder in Halo's installation subdirectory, then copy `hlef.dll` to it;
 * for `sapp` users, copy `hlef.dll` to the same directory as `sapp.dll` and add (this script)[/lua/bsp_exorcist.lua] to your configuration.

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
that a BSP tree recursively subdivides space through partitioning hyperplanes (space 
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

The explanations below will assume that double-sided surfaces are not present, for 
the sake of brevity.

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
[con's ghostbuster](https://opencarnage.net/index.php?/topic/8069-ghostbuster-a-phantom-bsp-tag-fixer-deprecated/)
took. However, even in infinite precision, this method punches holes (different from 
BSP leaks) into the map because *phantom BSP can occur over other surfaces*. 
Consider the following example.

![wizard_phantom_bsp](/wizard_phantom_bsp.png)

 * The red arrow into the face indicates where the vector intersects with the surface.
 * The red outline indicates the surface intersected. 
 * The lime-green line indicates the top-bottom leaf split in the BSP. 

As shown, excess geometry is attributed to the bottom surface, outside of the 
surface bounds.It may be that this is a result of an intentional optimization 
employed by `tool.exe`, but incorrectly implemented. However, this view is difficult 
to embrace, as some systems such as decal placement require a valid surface.

## Why do BSP leaks occur?
Consider the case that a leaf contains no 2D BSP reference associated with a plane 
that partitions BSP interior from exterior. Halo will not find an intersection in 
this case, and thus the BSP contains a leak. This issue is a bit more troubling to 
mitigate, because the leaf may not even reference a 'close enough' 2D BSP to fall 
back on. Consider the giant leak on Derelict.

![hole larger than sagittarius A*](/carousel_bsp_leak.png)
 
 * The area outlined in red is `surface #138`, `plane #45`.
 * The area marked in orange is `leaf #648`, and features a BSP leak.
 * The area marked in green is `leaf #635`, and features no BSP leak. It should be noted that this leaf is _EXTREMELY_ slim.

Suppose a line is intersecting `surface #138` through `leaf #635` in the 
green-highlighted area. The partitioning plane crossed is `plane #45`. The 
subroutine will find a BSP2D reference associated with `plane #45` in this leaf and 
correctly determine that the surface intersected is `surface #138`.

Now suppose a line is intersecting `surface #138` through `leaf #648` in the 
orange-highlighted area. The partitioning plane crossed is `plane #45`. The 
subroutine will find no BSP2D reference in this leaf associated with `plane #45`, 
and thus no intersection takes place. Distressingly, there is no BSP2D referenced 
in this leaf whose associated plane is even loosely coplanar with `plane #45`.

In fact, the traversal algorithm might not even be looking for `plane #45`, but a 
different plane that is nearly coplanar with `plane #45`.

To complicate matters, the BSP leak on Prisoner is an entirely different beast that 
I have trouble properly classifying. As far as I can tell, Prisoner's leak 
occurs because `tool` has generated a tunnel from the ladder to the ramp entrance 
below it. To put it another way, there is a volume that exists outside of the 
volume enclosed by the map geometry that bridges two apparently non-adjacent leaves.

To summarize, there are three forms of BSP leaks that I have encountered:
 * Form 1 BSP leaks occur when the leaf has a BSP2D reference associated with the correct surface, but the traversal algorithm is looking under the wrong plane;
 * Form 2 BSP leaks occur when the leaf does not have a BSP2D reference associated with the correct surface, and the traversal algorithm may or may not have the wrong plane;
 * Form 3 BSP leaks appear to be some sort of super leak that occurs when two non-adjacent leaves are somehow bridged.

Form 1 and 2 BSP leaks are trivially detected, but form 3 BSP leaks are quite 
sinister in that the leaking behaviour is consistent with valid formations in the 
tree. 

## TODOs
 * Reverse `collision_debug_phantom_bsp` to determine how Sapien detects phantom BSP
 * Convert to C++ (maybe)
 * Provide APIs to control mitigation behaviour