# Kinetica roadmap

## Design decisions

- **Scope**: 2D rigid bodies for games. Fixed, kinematic and dynamic bodies.
- **Units**: meters, kilograms, seconds. Y axis up. Rendering converts to pixels.
- **Precision**: `f32` everywhere.
- **Math**: `glam`, kept on the version macroquad uses so `Vec2` is shared without conversion.
- **API**: the `World` owns bodies and colliders; users hold copyable handles
  (`BodyHandle`, `ColliderHandle`) backed by a generational arena. Objects are created from
  `*Desc` builders.
- **Events** (to come): queued during `step`, read afterwards. No callbacks.
- **Determinism**: same inputs on the same build give bit-identical results, and
  `World: Clone` allows save/restore. Cross-platform determinism is a later goal.
- **Visibility**: only the intended API is `pub`; internals are `pub(crate)`.
- **Quality**: `unsafe` forbidden, clippy pedantic, documented public API, English only.
- **Engine independence**: the engine exposes a `DebugDraw` trait; renderers implement it.

## Phases

- [x] **0. Foundations**: workspace, math (`Rot`, `Transform`, `Aabb`), shapes (circle,
      convex polygon) with mass properties, bodies, colliders, integration, debug draw,
      macroquad testbed.
- [ ] **1. Collision detection**: broad phase (dynamic AABB tree), narrow phase
      (circle/polygon contact manifolds), contact begin/end events.
- [ ] **2. Contact solver**: soft-step solver with substepping, warm starting, friction and
      restitution. Goal: stable stacks and pyramids.
- [ ] **3. Queries**: ray casts, shape casts, point and AABB queries. Mouse dragging in the
      testbed.
- [ ] **4. Joints**: revolute, distance, prismatic, weld, mouse; motors and limits.
- [ ] **5. Game features**: sleeping, collision filtering (layers/masks), sensors,
      continuous collision detection, one-way platforms.
- [ ] **6. More shapes**: capsule, segment, chain (terrain).
- [ ] **7. Games**: a `kinetica-macroquad` glue crate, first small games under `games/`,
      WASM builds, optional `serde` feature.
