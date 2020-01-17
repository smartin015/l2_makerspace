# Reverse Engineering Tool for 3D structures

## Purpose

When given a 3D model (e.g. STL file) come up with the most likely set of techniques used to produce it. 

A simpler version of this problem: given a 3d model made by a single tool (and the tool used), determine
what operations would produce a matching part.

## Ideas

Start with basic additive manufacturing (3D printing), and treat as a "volume matching" problem.
Consider a sequence of a limited set of operations (like the Fusion360 timeline):

- Add
- Move
- Scale

A neural net takes as input a scaled volume of the area of least overlap, plus metadata describing the features.
Its output is the next operation to make, applied to one of the features (or a new feature).

The actual training process requires evaluation in a 3d environment.
