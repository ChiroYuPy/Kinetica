# Kinetica roadmap

## Design decisions

- **Scope**: 2D rigid bodies for games. Fixed, kinematic and dynamic bodies.
- **Units**: meters, kilograms, seconds. Y axis up. Rendering converts to pixels.
- **Precision**: `f32` everywhere.
- **Math**: `glam`, kept on the version macroquad uses so `Vec2` is shared without conversion.
- **API**: the `World` owns bodies, colliders and joints; users hold copyable handles backed
  by a generational arena. `world[handle]` reaches the object and panics if it was removed;
  `get_body`/`get_collider`/`get_joint` return an `Option`. Objects are created from `*Desc`
  builders: a `BodyDesc` carries its colliders, so one call creates a whole body. Sizes are
  full sizes (`rectangle(width, height)`, `vertical_capsule(height, radius)`), and a collider
  is placed on its body with `offset` and `angle`. On/off settings are argument-free builders
  (`sensor()`, `bullet()`, `lock_rotation()`, `keep_awake()`, `closed()`).
- **Joints API**: anchors are world points, attached to the bodies where they are when the
  joint is added; the joint angle or translation is zero in that placement. `add_joint`
  returns a typed handle (`JointHandle<RevoluteJoint>`), so `world[motor].set_motor(...)`
  needs no matching; `Joint` is an enum for joints of any type, and `untyped()` stores
  handles of different types together.
- **Setters**: setting a value a body already has does nothing, so game code may set
  velocities every step without keeping bodies awake.
- **Events**: queued during `step`, read afterwards with `World::collision_events`. No
  callbacks. `Started` carries the impact speed (fastest approach speed along the normal at
  the contact points). Removing a collider or body does not report `Stopped` for its contacts.
- **Materials**: combined friction is `sqrt(a * b)`, combined restitution is `max(a, b)`.
- **Known limit**: simultaneous contacts are solved together, so a row of touching bodies
  shares an impact instead of passing it along (a touching Newton's cradle fails). A small
  gap between bodies makes impacts sequential and fixes it.
- **Stiffness**: contacts are 30 Hz springs and joints 60 Hz ones with the default 4 substeps
  at 60 steps per second; shorter substeps make both stiffer. With 4 substeps a single column
  of 18 boxes leans on its contacts until it topples (pyramids and wider stacks stand); with 8
  substeps it stands, and with 16 a column of 30 does.
- **Joints**: soft like contacts (60 Hz, damping ratio 2), so long rigid assemblies sag a
  little under load (an 8 m welded cantilever drops about 0.7 m at its tip). Joined bodies do
  not collide unless `collide_connected` is set. `Joint::constraint_force` and
  `constraint_torque` report the load of the last step (stored impulse / substep), which is
  enough for breakable joints and tearing cloth.
- **Soft structures**: cloth, ropes and soft bodies are built from small bodies and joints in
  the testbed, not special engine objects. In 2D, things moving in the cloth plane push it
  rather than pierce it, so cloth only tears from load (or a cutting tool).
- **Queries**: rays starting inside a shape do not hit it; a shape cast starting in contact
  hits at fraction 0. Set queries return a sorted `Vec`. Queries skip sensors by default.
- **Sleeping**: islands of dynamic bodies linked by touching contacts and joints sleep
  together after 0.5 s below 0.05 m/s and 2°/s, and wake together. Fixed and kinematic
  bodies do not link islands; a moving kinematic body keeps what it touches awake, and only
  sleeps itself at exactly zero velocity. Setters and removals wake the bodies concerned.
- **Filtering**: 32 layers, `memberships`/`filter` masks checked both ways, re-evaluated
  every step so they can change at runtime.
- **Sensors**: separate overlap tracking with `Entered`/`Exited` events, no mass, never
  detect other sensors.
- **One-way platforms**: a collider flag; whether a contact blocks is decided when it
  starts (normal within 45° of the platform direction) and kept until it ends.
- **Continuous collision**: a dynamic body moving more than half its thickness in a step is
  swept (translation only, final orientation) against fixed and kinematic colliders, and
  moved back to the time of impact; bullets also sweep against dynamic bodies.
- **Shapes**: circle, polygon (up to 8 vertices), capsule, segment and chain segment.
  Everything but the circle is a *rounded convex core* in the narrow phase (Box2D v3 style):
  SAT and clipping with radii, a closest-points pass for rounded corners, and a dedicated
  routine for two segment cores (SAT misses the axis along them). Segments have no mass.
- **Chains**: one collider per segment, created by `World::add_chain`. One-sided, colliding
  on the left of their direction (ground listed left to right; closed outlines clockwise for
  solids, counter-clockwise for containers). Each segment knows its neighbors and only
  accepts contact normals they make possible, so shapes slide across joints without
  catching; queries still hit them from both sides.
- **Contacts**: only created when at least one body is dynamic, never between colliders of the
  same body. Points are reported slightly before shapes touch (speculative distance).
- **Determinism**: same inputs on the same build give bit-identical results, and
  `World: Clone` allows save/restore. Cross-platform determinism is a later goal.
- **Serialization**: the optional `serde` feature covers the whole world state and every
  description, so worlds can be saved and a loaded world continues bit for bit like the
  original. No state field holds an infinity (JSON cannot store it).
- **Games**: small demos with simple, cut-paper graphics (`kinetica_macroquad::paper`: paper
  sheet, shapes with an ink edge and a soft shadow, a handwritten font). `kinetica-macroquad`
  holds what every macroquad game needs; game rules are kept apart from input and drawing so
  they can be tested headlessly (the slingshot and snip tests search for winning moves, the
  trials test drives the track with an autopilot). Web builds use the JavaScript loader shipped with the exact
  macroquad version, plus the scripts of plugins such as `quad-url`.
- **Visibility**: only the intended API is `pub`; internals are `pub(crate)`.
- **Quality**: `unsafe` forbidden, clippy pedantic, documented public API, English only.
- **Engine independence**: the engine exposes a `DebugDraw` trait; renderers implement it.

## History

Each phase was a branch, merged once reviewed; git has the details.

| # | Phase | Outcome |
|---|-------|---------|
| 0-3 | Foundations, collision, solver, queries | Bodies and shapes, dynamic AABB tree, SAT and clipping manifolds, soft-step solver (Box2D v3 style), ray and shape casts. A 210-box pyramid stands. |
| 4-6 | Joints, game features, more shapes | Revolute, distance, prismatic, weld and mouse joints; sleeping islands, layers, sensors, continuous collision, one-way platforms; capsules, segments and smooth one-sided chains. |
| 7-8 | Games and testbed | `kinetica-macroquad`, the first games, WASM builds, `serde`; a testbed with one scene per concept. |
| 9, 14 | Performance | `kinetica-bench` with checksums proving an optimization changes no result; surface-area tree rotations, contacts in sorted `Vec`s. Graph coloring and SIMD were set aside on purpose: much faster, much harder to read. |
| 10-12 | Readability | The API rewritten from the games' point of view; shared joint math, `docs/ARCHITECTURE.md`; game helpers moved to `kinetica-macroquad`. |
| 15 | Accuracy | Checked against textbook results (slopes, friction, rolling, bounces, elastic collisions); contacts and joints get stiffer with shorter substeps. |
| 16, 18 | Testbed trimmed | 29 scenes down to 24, every concept kept: merged ropes and bridges, layers and one-way platforms, ray and shape queries; the soft bodies scene shows four techniques (spokes, gas pressure, spring lattice, welded rim). |
| 17 | Games from scratch | `slingshot`, `trials`, `platformer`, `snip`, with one cut-paper look; the engine gained `WheelJoint`. |
| 18 | Slimming | The `profile` feature removed (`perf` does the job), `Circle::with_center` made internal, this history condensed. |
| 19 | Bounces | Restitution uses the approach speed reached during the step, not the one at its start: bounces at e = 0.8 lose 0.5 % of height instead of 1.3 %. A velocity-dependent speculative distance stopped balls without sinking but made fast bodies hit things they would miss, so it was dropped. |
| 20 | Soft bodies | The balloon gets air drag, which also stops its skin from shaking forever; a new pool scene piles up gas-filled bodies made of welded capsules, which flatten but never collapse. |
