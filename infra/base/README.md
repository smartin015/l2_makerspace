# Title

## Objective

Provide a stable foundation for L2 docker containers - common libraries, boilerplate, message types etc.

## Message Types

### Generate

Generate challenge space

Common language for describing challenges

- Free-form textual challenges (GPT2-style synthesis)
- Static challenges (build a structure with constraints)
- Electrical challenges (build a circuit with constraints)
- Dynamic challenges (build a dynamic system with constraints)
- Textile challenges ()

Future... chemical challenges? artistic challenges? Must be extensible.

Generate solutions and make exploration easy

- Evolutionary style (select the ones to improve upon, iterate)

### Optimize

- 2.5D optimization (reduce weight on laser/plasma cut part)
- 3D optimization (e.g. minimize cost for parametric structure)

### Create

- Generate readable, stepwise instruction manual for assembling a result
- Common language for performing operations on parts (cut / join / fill etc)
- Common language for positioning materials around tools (load/unload/adjust etc)

### Infrastructure

- Common language for passing files / project & tasks etc.

## XML Schema (#xmlns)

There are several attributes applied to SDF files that are custom (following the [custom elements SDF proposal](http://sdformat.org/tutorials?tut=custom_elements_attributes_proposal&cat=pose_semantics_docs&)):

### Attributes

**l2:type** is an attribute that indicates the item described is managed by a custom object (in the L2 VR code).

Examples: *pendant*, *depth_render*, *control_zone*

**l2:control** is an attribute that indicates a preconfigured control scheme should be used for the item (in L2 VR code).

Examples: *test_control_preset*
