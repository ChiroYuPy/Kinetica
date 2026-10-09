# How Kinetica works

This is a map of the engine for people reading or changing its code. The public API is
documented in the crate docs (`cargo doc -p kinetica --open`); this page covers what happens
behind it.

## Where things live

Everything is in `crates/kinetica/src`.

| Module | Role |
|--------|------|
| `world.rs` | `World`: owns everything, adds and removes objects, runs a step. |
| `arena.rs` | Generational arena behind every handle. |
| `body.rs`, `collider.rs` | Bodies and colliders, with their `*Desc` builders. |
| `shape/` | Circle, polygon, capsule, segment, chain segment: geometry and mass. |
| `math/` | `Rot`, `Transform`, `Aabb`, and a 2×2 symmetric matrix; vectors come from glam. |
| `collision/broad_phase/` | Dynamic AABB tree finding colliders whose boxes overlap. |
| `collision/narrow_phase/` | Exact contact points between two shapes (`collide`). |
| `contact/` | Tracks every candidate pair from step to step; contact and sensor events. |
| `solver/` | Turns contacts into constraints and moves bodies (soft step). |
| `joint/` | The five joint types, and the math they share (`frame.rs`). |
| `island.rs` | Groups of touching bodies that fall asleep and wake up together. |
| `continuous.rs` | Stops fast bodies from tunneling through thin obstacles. |
| `query.rs` | Ray casts, shape casts, point and area queries. |
| `constants.rs` | Every tolerance and tuning value, with why it has its value. |
| `debug_draw.rs`, `profile.rs` | Drawing the world for debugging; timing the stages of a step. |

## Handles

Bodies, colliders and joints are stored in `Arena`s. Adding one returns a handle wrapping an
arena `Index` (a slot and a generation). Removing a value bumps its slot generation, so an old
handle never reaches whatever reuses the slot. Objects refer to each other with handles too:
a collider knows its body, a body lists its colliders and joints.

## One step

`World::step` runs seven stages, numbered in its code:

1. **Contacts** (`contact/set.rs`). The contact set holds every pair of colliders whose
   *fat* boxes overlap. For each pair with an awake body, the narrow phase computes a fresh
   *manifold*: a normal and up to two points, each with its separation. Points are reported
   slightly before the shapes touch (`SPECULATIVE_DISTANCE`), so the solver can stop bodies
   before they overlap. Impulses of points that persist are copied from the last step (warm
   starting). Pairs that start or stop touching are reported as events. Sensors use the same
   pairs, but only test for overlap.
2. **Islands** (`island.rs`). Dynamic bodies linked by touching contacts or joints form
   islands, found with a union-find. An island with any awake body wakes up entirely.
3. **Solver** (`solver/`). The step is split into substeps (4 by default). In each substep:
   apply gravity and forces to velocities; reapply the last impulses (warm start); *solve*
   joints, then contacts, pushing overlaps apart softly; move the bodies; then *relax*, solving
   again without the push so the bodies do not fly apart. Restitution is applied once at the
   end. This is Box2D v3's "soft step" solver.
4. **Synchronize**. Collider boxes follow their bodies. A collider whose box leaves its fat
   box is re-inserted in the broad phase tree and marked as moved.
5. **Continuous collision** (`continuous.rs`). A body that moved more than half its
   thickness is swept along its motion; if it would have passed through something, it is
   moved back to the first time of impact. The contact found at the next step stops it.
6. **Sleep**. Islands whose bodies all stayed nearly still for half a second fall asleep:
   the solver and the contact update skip them until something wakes them.
7. **New pairs** (`collision/broad_phase/`). The broad phase looks for new overlaps around
   the colliders that moved, and the contact set starts tracking them. Their geometry is
   computed at stage 1 of the next step.

## Soft constraints

Contacts and joints are constraints: rules on the relative motion of two bodies, such as
"these points do not get closer" or "these anchors stay together". Each substep, a constraint
computes the impulse that fixes its velocity error, plus a *bias* that also corrects part of
its position error. Rigid corrections jitter and explode; Kinetica treats every constraint as
a stiff damped spring instead (`solver/softness.rs`). A `Softness` turns a frequency and a
damping ratio into the three numbers the impulse formula needs (`Softness::impulse`). A few
patterns recur:

- **Accumulated impulses.** Each constraint keeps the total impulse it applied during the step
  and clamps that total, not each increment: a contact can push but never pull, friction is
  bounded by the normal impulse, a motor by its force. The total also warm-starts the next
  step.
- **Solve and relax passes** (`solver::Pass`). The solve pass corrects positions; the relax
  pass only fixes velocities, removing the speed the correction added.
- **Limits** (`joint/frame.rs`). Joint limits and contacts behave the same way: while there is
  room, they only stop the motion that would cross them within this substep; once crossed,
  they push back softly.
- **Anchors at the start of the step** (`JointFrame`). Joints capture their anchors and masses
  once per step, then rotate the anchors by how far each body turned. They do not recompute
  everything at each substep.

## Determinism

The same build, given the same calls, produces bit-identical results. Arenas iterate in slot
order, contacts live in lists sorted by collider handles, and every sort uses a total
order with handles as tie-breakers. `kinetica-bench` prints a checksum for each scenario:
a change that should not alter the simulation, like a refactor or an optimization, must leave
them unchanged.
