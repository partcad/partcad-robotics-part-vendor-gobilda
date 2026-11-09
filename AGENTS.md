# goBILDA parts for PartCAD

This package is `//pub/robotics/parts/gobilda`: goBILDA catalog items as PartCAD parts, each read from the STEP
model goBILDA publishes on the item's product page, with the ports and interfaces it is connected through.

Everything below assumes PartCAD is installed and `pc` is on the `PATH`. In a checkout of
[partcad](https://github.com/partcad/partcad) that is `poetry run pc`, and `poetry run python` is a Python
that has PartCAD and OpenCASCADE (`OCP`) in it. Pass `--no-ansi` to `pc` when reading its output from a script.

## Adding a part

1. Download the item's STEP file from its product page on `www.gobilda.com` and put it where its name says,
   under `hardware/`, `motion/` or `structure/`: `motion/hub_sonic_8mmREX.step` is `motion/hub_sonic_8mmREX`.
2. Declare it in `partcad.yaml`, in alphabetical order among the others:

   ```yaml
   parts:
     motion/hub_sonic_8mmREX:
       desc: 1309 Series Sonic Hub (8mm REX Bore)   # the product's name, as the page has it
       vendor: gobilda
       sku: 1309-0016-4008                          # the SKU on the page
       count_per_sku: 2                             # only for a pack of several ("- 2 Pack")
       url: https://www.gobilda.com/1309-series-sonic-hub-8mm-rex-bore/
       type: step
   ```

   A SKU that is a *set* of different items - a REX shaft comes with an E-clip - is one part per item, each naming
   the set's SKU and which item of it it is with `item_in_sku`. An item that comes in several sets (the same clip
   comes with every shaft length) is declared once for its geometry, and once more per set as an `alias` naming
   that set's SKU: see `motion/shaft_8mmREX_clip` and the aliases after it.

3. Say what it is made of. The STEP file has no colour or material in it, so take them from the spec table on the
   product page, using the `colors:`, `materials:` and `finishes:` declared at the top of `partcad.yaml`:

   ```yaml
       parameters:
         material: {type: string, default: ":6061-T6 Aluminum"}  # the grade of the page's "Material"
         finish: {type: string, default: ":Clear Anodized"}      # its "Finish", if it states one
         color: {type: string, default: ":Zinc-Plated"}          # only where the material's own colour is hidden
   ```

   The page names a metal ("Aluminum", "Steel"), never a grade, so a material here is a guess at the grade, named
   the way the grade is specified (`6061-T6 Aluminum`, `303 Stainless Steel`): the grade such an item is usually
   made of, checked against the page's "Weight". A material claims the colour it looks, so a part made of it names
   no colour unless something covers the metal: a plating, or a colour the page states ("Color: Black"). Add a
   declaration for a grade, colour or finish the package does not have yet.
   Then compare the mass `pc info` reports with the page's "Weight". If they are more than 10% apart, try the
   grades the page's metal could be before anything else. If none accounts for it, goBILDA's weight is the one
   that counts: state it under `properties: physics: mass:` (in kg), and say in a comment what the model weighs,
   how far that is off, and the plausible reasons - the material information is incomplete or wrong, the model
   has voids the item does not (or the other way round), the listed weight includes something the model leaves
   out. Where the model is simplified on purpose (a bearing drawn as a solid ring with no balls), say just that.
   Something that is no one material, a servo, states goBILDA's weight too, and so does an assembly whose pieces
   do not add up to it.

   Last, require it: `requirements: {has_color: true, has_material: true, has_weight: true}` (only `has_weight` for
   what is no one material), which `pc test` checks.

   **A model that is pieces of different materials** - a hub and its steel pinch bolts, one solid each in one STEP
   file - is an assembly of those pieces. See `motion/hub_sonic_8mmREX` and its `.assy` file:

   * `<name>_model` reads the file as it is (`path:` it), and is `manufacturable: false`: it is no item.
   * Each kind of piece but the main one is `<name>_<piece>`, a `boolean` part: `operation: and` of the model and
     `<name>_<piece>_box`, with `whole: true`, which keeps the model's solids that lie wholly inside the box. The
     box is a `basic` cuboid around those solids, a few tenths of a millimetre clear of them, that holds no other
     solid whole.
   * The main piece is `<name>_<body>`: the model `xor` the other pieces, `whole: true`, which is what is left of
     the model without them.
   * Each piece says what it is made of, as a part does, and is `manufacturable: false` too: it comes in the item.
   * The assembly `<name>` links the pieces, all at the origin (each is where the model has it), and carries what
     the part would have: `desc`, `vendor`, `sku`, `count_per_sku`, the requirements and the `implements:`. An
     assembly takes no `url:`, so the product page is a comment. An ASSY file links it as `assembly:`.

   `tools/deduce_ports.py --part <name>` reads such an assembly's model and checks it against the assembly's ports.

4. Deduce its ports:

   ```shell
   poetry run python tools/deduce_ports.py --part motion/hub_sonic_8mmREX
   ```

   This prints the `implements:` section the geometry calls for, leaving out whatever the part declares already.
   Read it before pasting it under the part: see "What the script cannot know" below. Lines starting with `#` are
   what the script saw and could not name - a spline, a D-bore, a size no standard names - and are for you to
   declare by hand, or to leave out.

5. Check it: `pc test` and `pc render`, and `pc lint`. All three have to pass. `pc render` redraws the drawings in
   `examples/` that the README shows, and those are kept in Git LFS, as PartCAD keeps its own (see
   `.gitattributes`): install `git-lfs` and run `git lfs install` in the clone before committing one. Without it
   git stores the drawing as it is where a pointer belongs, and says nothing. The STEP files stay in git itself:
   PartCAD clones a package without fetching LFS, and a part has to be read from a real file.

## The conventions

`partcad.yaml` spells them out at the top, and `tools/deduce_ports.py` follows them. In short:

* An opening's port is on the face the opening starts at, +Z into the material; a through hole has one instance
  per mouth. A shaft's port is where it leaves the part it sticks out of (or at both ends, for a part that is a
  shaft), +Z away from the shaft. Connecting two ports turns one of them around, which is what puts a shaft in a
  bore rather than beside it.
* A slot's port is the centre of one end, +X along the slot to the other end.
* An 8mm REX(TM) port's +X points at a corner of the hex. The `8mmREX` interface turns it by 15 degrees, which is
  what makes two REX ports meet with their flats aligned.
* Every tooth of a gear is a port, and so is every gap between two teeth: on the pitch circle (one module below the
  tips), +Z towards the gear's axis, +X along the tooth. `gear-tooth;module=<m>` mates `gear-gap;module=<m>`, so any
  tooth goes in any gap of a gear of the same module. A round gear implements `gear-<teeth>t;module=<m>,shift=<x>`
  once, on its axis, and gets every tooth and gap, numbered: see `motion/worm_gear_28t`, and `motion/worm_8mmREX`
  for a worm.
* Holes and shafts are the interfaces of `//pub/std/metric/m`: `m4-thru-8` where the standard names the size and
  the depth, `m-thru-depth;depth=8.5,size=4` where it does not. Write the parameters of such a reference in
  alphabetical order: that is how PartCAD names it, and an ASSY file's `to:` is matched against that name.

## What the script cannot know

* **Whether a hole is tapped.** It is when the model draws the thread (its minor diameter inside it). goBILDA
  draws some tapped holes at their major diameter and nothing else, and those come out as clearance holes: the
  pattern mounts' holes are, for one. The product page says which holes are tapped.
* **What a feature is for**, beyond its shape. A 32mm pocket is an `m32` opening whether or not anything that
  size is ever put in it. Leave out what nothing will ever connect to.
* **Teeth.** It does not see them. Measure how far the tips of the teeth are from the axis, halfway across them, and
  where the first one is, and declare the gear by hand: the pitch circle is one module in from the tips, and whatever
  it is off `module * teeth / 2` by is its `shift`, in modules. A gear of a number of teeth nothing here has yet needs
  that number added to the list `gear-<teeth>t` is generated for.
* **Instance names.** It numbers the features and names each mouth by the face it is on (`h3-top`, `rex1-bottom`,
  `slot7-left`). Rename them to something a person connecting the part would recognise - `ledged` and `flat` on
  a hub - where there are few enough of them to be worth it; keep them as they are on a channel with hundreds.

To check a declaration against the model, `--all` prints everything the model says whether it is declared or not,
and `tools/deduce_ports.py <file>.step` does the same for a STEP file that is not a part of the package yet.
`--every` runs it over every part of the package.
