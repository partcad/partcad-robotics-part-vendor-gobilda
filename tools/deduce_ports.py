#!/usr/bin/env python3
#
# PartCAD, 2026
#
# Licensed under Apache License, Version 2.0.
#
"""Deduce the ports and interfaces of a goBILDA part from its STEP model.

    python tools/deduce_ports.py motion/hub_sonic_8mmREX.step
    python tools/deduce_ports.py --part motion/hub_sonic_8mmREX
    python tools/deduce_ports.py --part motion/hub_sonic_8mmREX --all

What comes out is the 'implements:' section of the part, in the conventions
'partcad.yaml' spells out at the top of the file, ready to paste under the part.
What the script saw and could not name goes to stderr, prefixed with '#', so
that it is read rather than lost.

With '--part', the part is read from this package - its STEP file, and the ports
it already declares - and only what is not declared yet is printed: a port that
lands where one of the part's own ports already is, facing the same way, is
already declared. '--all' prints everything the geometry says regardless, which
is how to check a declaration by hand against the model. '--part' needs PartCAD
(run it with the Python PartCAD is installed in); a bare STEP file needs only
the OpenCASCADE bindings PartCAD depends on (OCP).

It reads the model the way partcad-ldraw reads an LDraw part: it looks for the
features a connection is made through, and gives each one a port on the
surface where the partner arrives, one instance per mouth:

* a round hole: an 'm<size>' opening of '//pub/std/metric/m' - threaded when
  the model draws the minor diameter inside it, a clearance hole otherwise,
  through when both ends are open and a blind hole of its depth when not. The
  port is on the face the hole starts at, +Z into the material.
* a slot (two half circles joined by two flats): a slotted through hole, the
  port at the centre of one end and +X along the slot.
* an 8mm REX(TM) bore or shaft (a hex 7mm across the flats, rounded off at
  8mm): '8mmREX-thru-<depth>' or '8mmREX-shaft', +X at a corner of the hex.
* a round shaft: 'm<size>-shaft-<length>', the port where the shaft meets the
  part it sticks out of (or at both ends, for a shaft that is the whole part),
  +Z away from the shaft.
* the goBILDA hub pattern: four M4 holes on a 16mm square around a bore, as
  '4xM4-16mm-pattern-*', when this package declares one of that depth.

What it does not know - a spline, a D-bore, a size no standard names - it
reports and leaves to the author. So is the question a model cannot answer:
goBILDA draws the tapped holes of some parts at their major diameter, with no
thread in them, and those come out as clearance holes. Check what the part's
product page says before pasting.
"""

import argparse
import math
import os
import sys
from collections import defaultdict

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.gp import gp_Pnt
from OCP.STEPControl import STEPControl_Reader
from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_FACE, TopAbs_IN, TopAbs_REVERSED, TopAbs_SOLID, TopAbs_VERTEX
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

M = "//pub/std/metric/m"
# The sizes and depths '//pub/std/metric/m' publishes a name for. Anything else
# is asked for by value from the parametric interface the names are aliases of.
M_SIZES = [1, 1.2, 1.4, 1.6, 2, 2.5, 3, 3.5, 4, 5, 6, 8, 10, 12, 14, 16, 20, 24, 30, 32, 36, 42, 48, 56, 64]
M_DEPTHS = [1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6, 7, 8, 9, 10, 12, 14, 16, 20, 22]
# ISO 273's medium clearance hole per size: a hole up to this much bigger than
# the size is a clearance hole for it.
CLEARANCE = {3: 3.4, 4: 4.5, 5: 5.5, 6: 6.6, 8: 9, 10: 11, 12: 13.5, 14: 15.5, 16: 17.5}
# The 8mm REX(TM) profile: a hex 7mm across the flats, its corners rounded off
# at 8mm.
REX_FLAT = 3.5
REX_ROUND = 4.0
# The depths this package declares an '8mmREX-thru-<depth>' for.
REX_DEPTHS = [0.7, 5, 8, 9.5, 10, 10.5, 22, 32, 33]
# The patterns this package declares, by the interface of their holes.
PATTERNS = {
    "m4-thru-1.5": "4xM4-16mm-pattern-thru-1.5mm",
    "m4-thru-8": "4xM4-16mm-pattern-thru-8mm",
    "m4-threaded-thru-4": "4xM4-16mm-pattern-threaded-thru-4mm",
}

TOL = 0.02  # mm: two numbers this close are the same number in these models
SLIVER = 0.95  # mm: a round hole or shaft shorter than this is a lip or a race, not a feature
REACH = 0.3  # mm: how far a chamfer may move a feature's end off the face it ends on


# --- vectors -------------------------------------------------------------------


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def add(a, b):
    return [x + y for x, y in zip(a, b)]


def mul(a, k):
    return [x * k for x in a]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    return [x / n for x in a]


def rotate(v, axis, degrees):
    """'v' turned about 'axis' (a unit vector) by 'degrees', right-handed."""
    a = math.radians(degrees)
    c, s = math.cos(a), math.sin(a)
    return add(add(mul(v, c), mul(cross(axis, v), s)), mul(axis, dot(axis, v) * (1 - c)))


def canonical(d):
    """One of the two directions of a line, always the same one."""
    for x in d:
        if abs(x) > 1e-6:
            return d if x > 0 else mul(d, -1)
    return d


def perpendicular(d):
    """A unit vector perpendicular to 'd': +X where it can be, +Y otherwise."""
    other = [1, 0, 0] if abs(d[0]) < 0.9 else [0, 1, 0]
    return unit(sub(other, mul(d, dot(other, d))))


def location(origin, z, x):
    """The OCCT location '[[x, y, z], [axis], angle]' of a frame given by its Z and X axes."""
    z = unit(z)
    x = unit(sub(x, mul(z, dot(x, z))))
    y = cross(z, x)
    m = [[x[i], y[i], z[i]] for i in range(3)]
    trace = m[0][0] + m[1][1] + m[2][2]
    angle = math.acos(max(-1.0, min(1.0, (trace - 1) / 2)))
    if angle < 1e-9:
        axis = [0, 0, 1]
    elif abs(angle - math.pi) < 1e-6:
        axis = [math.sqrt(max(0.0, (m[i][i] + 1) / 2)) for i in range(3)]
        if axis[0] > 1e-6:
            axis[1] = math.copysign(axis[1], m[0][1])
            axis[2] = math.copysign(axis[2], m[0][2])
        elif axis[1] > 1e-6:
            axis[2] = math.copysign(axis[2], m[1][2])
        axis = unit(axis)
    else:
        axis = unit([m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]])
    return [[_num(v, 3) for v in origin], [_num(v, 6) for v in axis], _num(math.degrees(angle), 6)]


def _num(value, digits):
    value = round(value, digits)
    if value == int(value):
        return int(value)
    return value


# --- the model -----------------------------------------------------------------


class Cylinder:
    def __init__(self, face):
        surface = BRepAdaptor_Surface(face)
        cylinder = surface.Cylinder()
        axis = cylinder.Axis()
        p, d = axis.Location(), axis.Direction()
        self.radius = cylinder.Radius()
        direction = [d.X(), d.Y(), d.Z()]
        self.direction = canonical(direction)
        flip = 1 if dot(direction, self.direction) > 0 else -1
        origin = [p.X(), p.Y(), p.Z()]
        # The point of the axis closest to the origin, which names the line
        self.point = sub(origin, mul(self.direction, dot(origin, self.direction)))
        # Where along the line the face is, and how much of a turn it spans
        v0 = dot(origin, self.direction) + flip * surface.FirstVParameter()
        v1 = dot(origin, self.direction) + flip * surface.LastVParameter()
        self.start, self.end = min(v0, v1), max(v0, v1)
        self.span = math.degrees(surface.LastUParameter() - surface.FirstUParameter())
        # Material outside the surface is a hole, inside it a shaft
        self.hole = face.Orientation() == TopAbs_REVERSED
        # Which side of the axis the face is on, for the half circles of slots
        u = (surface.FirstUParameter() + surface.LastUParameter()) / 2
        v = (surface.FirstVParameter() + surface.LastVParameter()) / 2
        mid = surface.Value(u, v)
        self.middle = [mid.X(), mid.Y(), mid.Z()]

    def at(self, v):
        return add(self.point, mul(self.direction, v))


class Plane:
    def __init__(self, face):
        surface = BRepAdaptor_Surface(face)
        plane = surface.Plane()
        n, p = plane.Axis().Direction(), plane.Location()
        self.normal = [n.X(), n.Y(), n.Z()]
        self.point = [p.X(), p.Y(), p.Z()]
        self.vertices = []
        explorer = TopExp_Explorer(face, TopAbs_VERTEX)
        while explorer.More():
            v = BRep_Tool.Pnt_s(TopoDS.Vertex_s(explorer.Current()))
            self.vertices.append([v.X(), v.Y(), v.Z()])
            explorer.Next()


class Model:
    def __init__(self, path):
        reader = STEPControl_Reader()
        if reader.ReadFile(path) != 1:
            raise Exception("Cannot read %s" % path)
        reader.TransferRoots()
        self.shape = reader.OneShape()
        self.cylinders = []
        self.planes = []
        explorer = TopExp_Explorer(self.shape, TopAbs_FACE)
        while explorer.More():
            face = TopoDS.Face_s(explorer.Current())
            kind = BRepAdaptor_Surface(face).GetType()
            if kind == GeomAbs_Cylinder:
                self.cylinders.append(Cylinder(face))
            elif kind == GeomAbs_Plane:
                self.planes.append(Plane(face))
            explorer.Next()
        self.solids = []
        explorer = TopExp_Explorer(self.shape, TopAbs_SOLID)
        while explorer.More():
            self.solids.append(explorer.Current())
            explorer.Next()

    def inside(self, point):
        """Whether 'point' is in the material of the part."""
        for solid in self.solids:
            classifier = BRepClass3d_SolidClassifier(solid, gp_Pnt(*point), 1e-6)
            if classifier.State() == TopAbs_IN:
                return True
        return False

    def ring(self, line, v, radius):
        """The axis at 'v', and four points around it at 'radius'."""
        x = perpendicular(line.direction)
        y = cross(line.direction, x)
        centre = line.at(v)
        return [centre] + [add(centre, mul(w, radius * k)) for w in (x, y) for k in (1, -1)]

    def air(self, line, v0, v1, radius):
        """Whether the column of 'radius' around 'line' is open from 'v0' to 'v1'.

        Asked of the stretch between two pieces of one hole, which is one hole
        where another hole crosses it and two - a bearing's races - where
        there is material between them.
        """
        for v in (v0 + 0.05, (v0 + v1) / 2, v1 - 0.05):
            if any(self.inside(p) for p in self.ring(line, v, max(radius - 0.05, 0))):
                return False
        return True

    def opens(self, line, v, outward, radius):
        """Whether a hole of 'radius' ending at 'v' opens there, rather than stopping.

        Asked at the wall rather than on the axis: a tapped hole's tap drill
        goes on past the thread, and is not where the hole ends.
        """
        return not any(self.inside(p) for p in self.ring(line, v + outward * 0.05, radius * 0.9)[1:])

    def settle(self, axis_point, direction, v, outward):
        """Where a feature ending at 'v' really ends: on the face it ends on.

        A chamfer or an undercut at the mouth of a hole leaves the cylinder
        short of the face the hole is in. The face is the nearest plane across
        the axis within REACH of 'v', in the direction the feature ends in.
        """
        best = v
        for plane in self.planes:
            if abs(abs(dot(plane.normal, direction)) - 1) > 1e-4:
                continue
            w = dot(plane.point, direction)
            if 0 <= (w - v) * outward <= REACH and abs(w - v) >= abs(best - v):
                best = w
        return best


# --- features ------------------------------------------------------------------


def _lines(cylinders):
    """The cylinders grouped by the line they are around."""
    lines = []
    for c in cylinders:
        for line in lines:
            first = line[0]
            if norm(sub(c.direction, first.direction)) < 1e-4 and norm(sub(c.point, first.point)) < TOL:
                line.append(c)
                break
        else:
            lines.append([c])
    return lines


def _runs(faces, model=None):
    """Faces of one radius on one line, grouped into the stretches they cover.

    Two stretches with nothing but air between them are one: a hole crossed by
    another one is one hole, whatever the model cut it into.
    """
    runs = []
    for f in sorted(faces, key=lambda f: f.start):
        if runs and (
            f.start <= runs[-1]["end"] + TOL
            or (
                model is not None
                and f.hole
                # a hole crossing this one, not the space between two walls
                and f.start - runs[-1]["end"] <= 5 * f.radius
                and model.air(f, runs[-1]["end"], f.start, f.radius)
            )
        ):
            runs[-1]["end"] = max(runs[-1]["end"], f.end)
            runs[-1]["faces"].append(f)
        else:
            runs.append({"start": f.start, "end": f.end, "faces": [f]})
    for run in runs:
        run["span"] = min(360.0, sum(f.span for f in run["faces"]))
    return runs


def _overlap(c, start, end):
    """How much of 'start'..'end' the cylinder 'c' covers."""
    return max(0.0, min(c.end, end) - max(c.start, start))


def _covered(cylinders, start, end):
    """How much of 'start'..'end' the cylinders cover between them.

    A thread is modelled as a great many short faces, not as one.
    """
    covered, reach = 0.0, start
    for c in sorted(cylinders, key=lambda c: c.start):
        lo, hi = max(c.start, reach), min(c.end, end)
        if hi > lo:
            covered += hi - lo
            reach = hi
    return covered


def _size(diameter, hole):
    """The metric size a round feature of this diameter is for, or None."""
    for size in M_SIZES:
        if abs(diameter - size) <= 0.06:
            return size
        if hole and size in CLEARANCE and size < diameter <= CLEARANCE[size] + 0.05:
            return size
    return None


def _depth_name(depth):
    return ("%g" % round(depth, 3)).rstrip(".")


def _m_interface(kind, size, depth):
    """The '//pub/std/metric/m' interface of an M<size> feature this deep."""
    depth = round(depth, 3)
    if size in M_SIZES and depth in M_DEPTHS:
        return "%s:m%s-%s-%s" % (M, _depth_name(size), kind, _depth_name(depth))
    parametric = {
        "thru": "m-thru-depth",
        "threaded-thru": "m-threaded-thru-depth",
        "hole": "m-hole",
        "threaded-hole": "m-threaded-hole",
        "shaft": "m-shaft-length",
    }[kind]
    what = "length" if kind == "shaft" else "depth"
    # Parameters in alphabetical order, which is how PartCAD names an instance
    return "%s:%s;%s=%s,size=%s" % (M, parametric, what, _depth_name(depth), _depth_name(size))


FACES = [
    ([1, 0, 0], "right"),
    ([-1, 0, 0], "left"),
    ([0, 1, 0], "back"),
    ([0, -1, 0], "front"),
    ([0, 0, 1], "top"),
    ([0, 0, -1], "bottom"),
]


def _face_name(outward):
    for d, name in FACES:
        if dot(d, outward) > 0.999:
            return name
    return "side"


class Port:
    def __init__(self, interface, instance, origin, z, x, note):
        self.interface = interface
        self.instance = instance
        self.origin = origin
        self.z = unit(z)
        self.x = x
        self.note = note
        # Other places the same connection could be declared at: the other
        # end of a slot
        self.alternates = []


def deduce(model):
    """Every port the model says it has, and what it could not name."""
    ports = []
    unknown = []
    lines = _lines(model.cylinders)
    used = set()

    # Where each line's REX hex is, if it has one: flats 3.5mm off the axis,
    # parallel to it, facing at least three ways
    def hex_flats(line):
        c = line[0]
        flats = []
        for plane in model.planes:
            if abs(dot(plane.normal, c.direction)) > 1e-4:
                continue
            offset = sub(plane.point, c.point)
            if abs(abs(dot(offset, plane.normal)) - REX_FLAT) < TOL:
                flats.append(plane)
        facing = []
        for f in flats:
            if not any(abs(dot(f.normal, g)) > 0.999 for g in facing):
                facing.append(f.normal)
        return flats if len(facing) >= 3 else []

    feature = 0
    for line in lines:
        by_radius = defaultdict(list)
        for c in line:
            by_radius[(round(c.radius, 2), c.hole)].append(c)

        # 8mm REX(TM): a hex 7mm across the flats, around this line. A bore
        # may or may not draw its corners rounded; a shaft does, at 8mm, which
        # is what tells it from a 7mm hex nut.
        d = line[0].direction
        flats = hex_flats(line)
        if flats:
            along = [dot(v, d) for f in flats for v in f.vertices]
            start, end = min(along), max(along)
            middle = line[0].at((start + end) / 2)
            # A bore has nothing just inside its flats; a hex nut or a hex
            # shaft is solid there
            n = flats[0].normal
            toward = unit(sub(n, mul(d, dot(n, d))))
            if dot(sub(flats[0].point, line[0].point), toward) < 0:
                toward = mul(toward, -1)
            hole = not model.inside(add(middle, mul(toward, REX_FLAT - 0.1)))
            arcs = [c for c in line if abs(c.radius - REX_ROUND) < TOL and c.span < 359]
            if hole or arcs:
                feature += 1
                used.update(id(c) for c in arcs)
                start = model.settle(line[0].point, d, start, -1)
                end = model.settle(line[0].point, d, end, 1)
                # +X at a corner of the hex: 30 degrees off a flat
                n = flats[0].normal
                x = rotate(unit(sub(n, mul(d, dot(n, d)))), d, 30)
                if hole:
                    depth = end - start
                    named = min(REX_DEPTHS, key=lambda known: abs(known - depth))
                    interface = "8mmREX-thru-%s" % _depth_name(named)
                    note = "8mm REX bore, %.3fmm deep" % depth
                    if abs(named - depth) > 0.05:
                        note += " - no '8mmREX-thru-%s' is declared, add one" % _depth_name(depth)
                        interface = "8mmREX-thru-%s" % _depth_name(depth)
                    for v, into in ((start, 1), (end, -1)):
                        z = mul(d, into)
                        ports.append(
                            Port(interface, "rex%d-%s" % (feature, _face_name(mul(z, -1))), line[0].at(v), z, x, note)
                        )
                else:
                    note = "8mm REX shaft, %.3fmm long" % (end - start)
                    for v, out in ((start, -1), (end, 1)):
                        z = mul(d, out)
                        ports.append(
                            Port("8mmREX-shaft", "rex%d-%s" % (feature, _face_name(z)), line[0].at(v), z, x, note)
                        )

        for (radius, hole), faces in sorted(by_radius.items()):
            faces = [f for f in faces if id(f) not in used]
            if not faces:
                continue
            for run in _runs(faces, model):
                if run["span"] < 359:
                    continue  # half a slot, or a rounded edge: see '_slots'
                diameter = 2 * radius
                size = _size(diameter, hole)
                start = model.settle(line[0].point, d, run["start"], -1)
                end = model.settle(line[0].point, d, run["end"], 1)
                length = end - start
                if length < SLIVER:
                    continue
                if size is None:
                    # A hole is what something else is put into, and worth
                    # saying so about. A shaft of no standard size is nearly
                    # always a boss, a race or a rim, and would drown it out.
                    if hole and length >= 1:
                        unknown.append(
                            "a %s %.3fmm across at %s along %s, %.3fmm long: no metric size"
                            % ("hole" if hole else "shaft", diameter, _fmt(line[0].at(start)), _fmt(d), length)
                        )
                    continue
                feature += 1
                used.update(id(f) for f in run["faces"])
                if hole:
                    # Threaded when the model draws the minor diameter in it
                    threaded = (
                        size <= 10
                        and _covered(
                            [c for c in line if c.hole and 0.75 * radius < c.radius < 0.88 * radius], start, end
                        )
                        > 0.6 * length
                    )
                    # ... and the minor diameter of a thread is not a hole of
                    # its own: it is the hole above, already counted
                    minor = (
                        radius < 5
                        and _covered(
                            [c for c in line if c.hole and radius / 0.88 < c.radius < radius / 0.75], start, end
                        )
                        > 0.6 * length
                    )
                    if minor:
                        continue
                    open_start = model.opens(line[0], start, -1, radius)
                    open_end = model.opens(line[0], end, 1, radius)
                    if not open_start and not open_end:
                        continue  # a closed cavity
                    thru = open_start and open_end
                    kind = ("threaded-" if threaded else "") + ("thru" if thru else "hole")
                    interface = _m_interface(kind, size, length)
                    note = "%.3fmm %s hole, %.3fmm deep" % (diameter, "tapped" if threaded else "round", length)
                    x = perpendicular(d)
                    for v, into, is_open in ((start, 1, open_start), (end, -1, open_end)):
                        if not is_open:
                            continue
                        z = mul(d, into)
                        ports.append(
                            Port(interface, "h%d-%s" % (feature, _face_name(mul(z, -1))), line[0].at(v), z, x, note)
                        )
                else:
                    # A shaft in a hole of its own size is a screw the model
                    # was drawn with, in the hole it is screwed into
                    if any(c.hole and 0 <= c.radius - radius < 0.3 and _overlap(c, start, end) > 0 for c in line):
                        continue
                    # ... and so, nearly always, is a short thin one: a set
                    # screw drawn in place, which is not where anything else
                    # connects
                    if diameter < 8 and length < 3:
                        continue
                    # A shaft connects where it leaves the part it sticks out
                    # of: a point just inside its wall, past the end, is in
                    # the material there
                    # (far enough past it that a chamfered tip is not)
                    ends = []
                    for v, out in ((start, -1), (end, 1)):
                        wall = add(line[0].at(v + out * REACH), mul(perpendicular(d), radius * 0.9))
                        ends.append((v, out, model.inside(wall)))
                    attached = [(v, out) for v, out, inside in ends if inside]
                    if not attached:
                        # the whole part: a port at either end
                        attached = [(v, out) for v, out, inside in ends]
                    interface = _m_interface("shaft", size, length)
                    note = "%.3fmm round shaft, %.3fmm long" % (diameter, length)
                    for v, out in attached:
                        # +Z away from the shaft
                        z = mul(d, out)
                        ports.append(
                            Port(
                                interface, "s%d-%s" % (feature, _face_name(z)), line[0].at(v), z, perpendicular(d), note
                            )
                        )

    slot_ports, slot_unknown, feature, slot_ends = _slots(model, used, feature)
    # A round hole at the end of a slot is that slot, drawn as a hole too
    ports = [p for p in ports if not any(norm(sub(p.origin, e)) < TOL for e in slot_ends)]
    ports.extend(slot_ports)
    unknown.extend(slot_unknown)
    ports.extend(_patterns(ports))
    return ports, unknown


def _slots(model, used, feature):
    """Two half circles, facing each other across the same stretch: a slot.

    Each half is paired with the nearest one that faces it, and only if there
    is nothing but air between them: two halves of two different slots face
    each other just as well, across the material between them.
    """
    ports, unknown, ends = [], [], []

    def bulge(c):
        return sub(c.middle, c.at(dot(c.middle, c.direction)))

    # An end of a slot may be modelled as one half circle or as two quarters
    arcs = []
    for c in model.cylinders:
        if not c.hole or id(c) in used or c.span > 190:
            continue
        for arc in arcs:
            first = arc[0]
            if (
                abs(first.radius - c.radius) < TOL
                and norm(sub(first.direction, c.direction)) < 1e-4
                and norm(sub(first.point, c.point)) < TOL
                and abs(first.start - c.start) < TOL
                and abs(first.end - c.end) < TOL
            ):
                arc.append(c)
                break
        else:
            arcs.append([c])
    halves = []
    for arc in arcs:
        span = sum(c.span for c in arc)
        if 170 < span < 190:
            half = arc[0]
            direction = unit(add(*[bulge(c) for c in arc])) if len(arc) > 1 else unit(bulge(half))
            half.middle = add(half.at(dot(half.middle, half.direction)), mul(direction, half.radius))
            halves.append(half)

    candidates = []
    for i, a in enumerate(halves):
        for j in range(i + 1, len(halves)):
            b = halves[j]
            if abs(a.radius - b.radius) > TOL or norm(sub(a.direction, b.direction)) > 1e-4:
                continue
            if abs(a.start - b.start) > TOL or abs(a.end - b.end) > TOL:
                continue
            travel = sub(b.point, a.point)
            if norm(travel) < TOL or abs(dot(travel, a.direction)) > TOL:
                continue
            along = unit(travel)
            if dot(bulge(a), along) >= 0 or dot(bulge(b), along) <= 0:
                continue
            candidates.append((norm(travel), i, j))
    taken = set()
    for distance, i, j in sorted(candidates):
        if i in taken or j in taken:
            continue
        a, b = halves[i], halves[j]
        along = unit(sub(b.point, a.point))
        side = cross(a.direction, along)
        v = (a.start + a.end) / 2
        middle = add(a.at(v), mul(along, distance / 2))
        probes = [middle, add(middle, mul(side, a.radius * 0.9)), add(middle, mul(side, -a.radius * 0.9))]
        if any(model.inside(p) for p in probes):
            continue
        # ... and the walls of a slot just past that: the two halves of a slot
        # are joined by straight sides, where the ends of a cutout are not
        walls = [add(middle, mul(side, a.radius + 0.1)), add(middle, mul(side, -(a.radius + 0.1)))]
        if not all(model.inside(p) for p in walls):
            continue
        taken.update((i, j))
        diameter = 2 * a.radius
        size = _size(diameter, True)
        d = a.direction
        start = model.settle(a.point, d, a.start, -1)
        end = model.settle(a.point, d, a.end, 1)
        depth = end - start
        if depth < SLIVER:
            continue
        if size is None:
            unknown.append("a slot %.3fmm wide at %s: no metric size" % (diameter, _fmt(a.at(start))))
            continue
        feature += 1
        width = size + distance
        interface = "%s:m-thru-depth-slotted;depth=%s,size=%s,width=%s" % (
            M,
            _depth_name(depth),
            _depth_name(size),
            _depth_name(width),
        )
        note = "%.3fmm slot, %.3fmm of travel, %.3fmm deep" % (diameter, distance, depth)
        for v, into in ((start, 1), (end, -1)):
            z = mul(d, into)
            port = Port(interface, "slot%d-%s" % (feature, _face_name(mul(z, -1))), a.at(v), z, along, note)
            port.alternates = [b.at(v)]
            ports.append(port)
            ends.extend([a.at(v), b.at(v)])
    return ports, unknown, feature, ends


def _patterns(ports):
    """Four M4 holes on a 16mm square around a bore or a shaft: a hub pattern.

    The pattern is on the face the holes open onto, centred on the axis of
    what they are around - which need not open onto the same face: a hub's
    bore runs on through the ledge in the middle of it.
    """
    found = []
    by_interface = defaultdict(list)
    for p in ports:
        short = p.interface.split(":")[-1]
        if short in PATTERNS:
            by_interface[short].append(p)
    axes = []
    for p in ports:
        if p.interface.split(":")[-1] in PATTERNS:
            continue
        if not any(norm(sub(p.origin, q)) < TOL and abs(dot(p.z, z)) > 0.999 for q, z in axes):
            axes.append((p.origin, p.z))
    n = 0
    done = set()
    for short, holes in by_interface.items():
        for origin, axis in axes:
            for facing in {tuple(h.z) for h in holes}:
                facing = list(facing)
                if abs(dot(facing, axis)) < 0.999:
                    continue
                around = []
                for h in holes:
                    if norm(sub(h.z, facing)) > 1e-4:
                        continue
                    offset = sub(h.origin, origin)
                    radial = sub(offset, mul(axis, dot(offset, axis)))
                    if abs(norm(radial) - 8 * math.sqrt(2)) < TOL:
                        around.append(h)
                planes = {round(dot(h.origin, facing), 3) for h in around}
                if len(around) != 4 or len(planes) != 1:
                    continue
                centre = add(origin, mul(axis, dot(sub(around[0].origin, origin), axis)))
                key = (short, tuple(round(v, 3) for v in centre), tuple(facing))
                if key in done:
                    continue
                done.add(key)
                n += 1
                # +X along a side of the square, the way the pattern is drawn
                corner = unit(sub(around[0].origin, centre))
                x = rotate(corner, facing, 45)
                face = _face_name(mul(facing, -1))
                pattern = Port(
                    PATTERNS[short], "p%d-%s" % (n, face), centre, facing, x, "four M4 holes on a 16mm square"
                )
                # Declared already where its holes are
                pattern.alternates = [h.origin for h in around]
                found.append(pattern)
    return found


def _fmt(v):
    return "[%s]" % ", ".join("%g" % round(x, 3) for x in v)


# --- what is declared already ----------------------------------------------------


class Package:
    """This package, as PartCAD reads it: its parts and the ports they declare."""

    def __init__(self, path):
        os.environ.setdefault("PC_TELEMETRY_TYPE", "none")
        import partcad as pc

        self.path = path
        self.ctx = pc.init(path)
        self.project = self.ctx.get_project(self.ctx.current_project_path)

    def step_parts(self):
        """The parts read from a STEP file, aliases included, by name."""
        return sorted(
            name for name, part in self.project.parts.items() if (part.config or {}).get("type") in ("step", "alias")
        )

    def step_file(self, part_name):
        part = self.project.get_part(part_name)
        if part is None:
            raise Exception("No part '%s' in %s" % (part_name, self.path))
        config = part.config or {}
        if config.get("type") == "alias":
            # The geometry is the source's, and so are the ports (see
            # 'PartFactoryAlias'): nothing to deduce for the alias itself
            return None
        return os.path.join(self.path, config.get("path") or part_name + ".step")

    def declared_ports(self, part_name):
        """Where the part's declared ports are, as (origin, z) pairs."""
        from partcad.shape_ports import own_ports

        result = []
        for record in own_ports(self.project.get_part(part_name)):
            loc = record.location
            result.append((list(loc.translation), list(loc.rotate_vector((0, 0, 1)))))
        return result


def covered(port, declared):
    for origin, z in declared:
        if dot(z, port.z) < 0.999:
            continue
        if any(norm(sub(origin, where)) < 0.05 for where in [port.origin] + port.alternates):
            return True
    return False


# --- output --------------------------------------------------------------------


def _unique(ports):
    """Instance names that are unique within each interface."""
    seen = defaultdict(int)
    for p in ports:
        key = (p.interface, p.instance)
        seen[key] += 1
        if seen[key] > 1:
            p.instance = "%s-%d" % (p.instance, seen[key])


def render(ports):
    _unique(ports)
    lines = []
    by_interface = defaultdict(list)
    for p in ports:
        by_interface[p.interface].append(p)
    for interface in sorted(by_interface):
        lines.append('"%s":' % interface if ":" in interface or ";" in interface else "%s:" % interface)
        seen = set()
        for p in sorted(by_interface[interface], key=lambda p: p.instance):
            if p.note not in seen:
                lines.append("  # %s" % p.note)
                seen.add(p.note)
            lines.append("  %s: %s" % (p.instance, location(p.origin, p.z, p.x)))
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("step", nargs="?", help="the STEP file to read")
    parser.add_argument("--part", help="a part of this package: read its STEP file and skip what it declares already")
    parser.add_argument("--every", action="store_true", help="every STEP part of this package, one after another")
    parser.add_argument("--package", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    parser.add_argument("--all", action="store_true", help="print what is declared already as well")
    args = parser.parse_args(argv)

    if args.step:
        ports, unknown = deduce(Model(args.step))
        for line in unknown:
            sys.stderr.write("# not recognized: %s\n" % line)
        if ports:
            print(render(ports))
        return

    if not args.part and not args.every:
        parser.error("name a STEP file, a --part, or --every")
    package = Package(os.path.abspath(args.package))
    for name in [args.part] if args.part else package.step_parts():
        path = package.step_file(name)
        if path is None:
            continue
        declared = [] if args.all else package.declared_ports(name)
        ports, unknown = deduce(Model(path))
        new = [p for p in ports if not covered(p, declared)]
        if args.every:
            print("\n### %s" % name)
        for line in unknown:
            print("# not recognized: %s" % line)
        if declared:
            print("# %d of the %d ports found are declared already" % (len(ports) - len(new), len(ports)))
        if new:
            print(render(new))


if __name__ == "__main__":
    main()
