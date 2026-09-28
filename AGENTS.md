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

3. Deduce its ports:

   ```shell
   poetry run python tools/deduce_ports.py --part motion/hub_sonic_8mmREX
   ```

   This prints the `implements:` section the geometry calls for, leaving out whatever the part declares already.
   Read it before pasting it under the part: see "What the script cannot know" below. Lines starting with `#` are
   what the script saw and could not name - a spline, a D-bore, a size no standard names - and are for you to
   declare by hand, or to leave out.

4. Check it: `pc test` and `pc render`, and `pc lint`. All three have to pass.

## The conventions

`partcad.yaml` spells them out at the top, and `tools/deduce_ports.py` follows them. In short:

* An opening's port is on the face the opening starts at, +Z into the material; a through hole has one instance
  per mouth. A shaft's port is where it leaves the part it sticks out of (or at both ends, for a part that is a
  shaft), +Z away from the shaft. Connecting two ports turns one of them around, which is what puts a shaft in a
  bore rather than beside it.
* A slot's port is the centre of one end, +X along the slot to the other end.
* An 8mm REX(TM) port's +X points at a corner of the hex. The `8mmREX` interface turns it by 15 degrees, which is
  what makes two REX ports meet with their flats aligned.
* Holes and shafts are the interfaces of `//pub/std/metric/m`: `m4-thru-8` where the standard names the size and
  the depth, `m-thru-depth;depth=8.5,size=4` where it does not. Write the parameters of such a reference in
  alphabetical order: that is how PartCAD names it, and an ASSY file's `to:` is matched against that name.

## What the script cannot know

* **Whether a hole is tapped.** It is when the model draws the thread (its minor diameter inside it). goBILDA
  draws some tapped holes at their major diameter and nothing else, and those come out as clearance holes: the
  pattern mounts' holes are, for one. The product page says which holes are tapped.
* **What a feature is for**, beyond its shape. A 32mm pocket is an `m32` opening whether or not anything that
  size is ever put in it. Leave out what nothing will ever connect to.
* **Instance names.** It numbers the features and names each mouth by the face it is on (`h3-top`, `rex1-bottom`,
  `slot7-left`). Rename them to something a person connecting the part would recognise - `ledged` and `flat` on
  a hub - where there are few enough of them to be worth it; keep them as they are on a channel with hundreds.

To check a declaration against the model, `--all` prints everything the model says whether it is declared or not,
and `tools/deduce_ports.py <file>.step` does the same for a STEP file that is not a part of the package yet.
`--every` runs it over every part of the package.
