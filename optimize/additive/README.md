# Additive manufacturing optimization

## Purpose

Present a standard interface for generating / optimizing 3D models for additive manufacturing (e.g. 3D printing).

## Ideas

Program takes a starter FreeCAD document that has handcrafted geometry for bits the user knows should exist
And a "stamp" such as a voxel, tube/pipe, gear, etc. to be replicated to produce the final result.

Note that in the case of gears, we could have the starter document be "Gear A spins clockwise" and "Gear B spins counter-clockwise"
with two very different gears at different locations. The direction of spin would be a constraint in addition to the geometrical constraints.

V2 idea: allow any stamp to interop with any other stamp in particular ways. 

An ml model reccomends 

