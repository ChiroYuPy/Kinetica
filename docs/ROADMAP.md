# Kinetica roadmap

## Design decisions

- **Scope**: 2D rigid bodies for games. Fixed, kinematic and dynamic bodies.
- **Units**: meters, kilograms, seconds. Y axis up. Rendering converts to pixels.
- **Precision**: `f32` everywhere.
- **Math**: `glam`, kept on the version macroquad uses so `Vec2` is shared without conversion.
- **API**: the `World` owns bodies and colliders; users hold copyable handles
  (`BodyHandle`, `ColliderHandle`) backed by a generational arena. Objects are created from
  `*Desc` builders.
- **Events**: queued during `step`, read afterwards with `World::collision_events`. No
  callbacks. Removing a collider or body does not report `Stopped` for its contacts.
- **Materials**: combined friction is `sqrt(a * b)`, combined restitution is `max(a, b)`.
- **Known limit**: simultaneous contacts are solved together, so a row of touching bodies
  shares an impact instead of passing it along (a touching Newton's cradle fails). A small
  gap between bodies makes impacts sequential and fixes it.
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
- **Contacts**: only created when at least one body is dynamic, never between colliders of the
  same body. Points are reported slightly before shapes touch (speculative distance).
- **Determinism**: same inputs on the same build give bit-identical results, and
  `World: Clone` allows save/restore. Cross-platform determinism is a later goal.
- **Visibility**: only the intended API is `pub`; internals are `pub(crate)`.
- **Quality**: `unsafe` forbidden, clippy pedantic, documented public API, English only.
- **Engine independence**: the engine exposes a `DebugDraw` trait; renderers implement it.

## Phases

- [x] **0. Foundations**: workspace, math (`Rot`, `Transform`, `Aabb`), shapes (circle,
      convex polygon) with mass properties, bodies, colliders, integration, debug draw,
      macroquad testbed.
- [x] **1. Collision detection**: broad phase (dynamic AABB tree with fat boxes), narrow
      phase (circle/polygon manifolds via SAT and clipping), deterministic contact set,
      `Started`/`Stopped` events, contact debug drawing.
- [x] **2. Contact solver**: soft-step solver (Box2D v3 style) with 4 substeps, relax pass,
      warm starting by feature id, Coulomb friction, restitution. A 210-box pyramid stands.
- [x] **3. Queries**: closest and all-hits ray casts, shape casts (conservative advancement),
      point, box and overlap queries, all with a `QueryFilter`; brute-force checked.
- [x] **4. Joints**: revolute (limits, motor), distance (rod, spring, rope), prismatic (limits,
      motor), weld (rigid or elastic), mouse. Joint-only scenes: pendulums, Newton's cradle,
      chains, bridge, ragdolls, suspended car, springs, crank-slider, elevator, cantilever.
- [x] **5. Game features**: sleeping by islands, collision layers, sensors with events,
      continuous collision (bullets), one-way platforms.
- [ ] **6. More shapes**: capsule, segment, chain (terrain).
- [ ] **7. Games**: a `kinetica-macroquad` glue crate, first small games under `games/`,
      WASM builds, optional `serde` feature.
