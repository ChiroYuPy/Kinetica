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
- [x] **6. More shapes**: capsule, segment, smooth one-sided chains for terrain; the car
      terrain, ragdolls and stairs use them.
- [x] **7. Games**: `kinetica-macroquad` glue crate, `slingshot` and `platformer` games,
      WASM builds of the games and the testbed, optional `serde` feature.
- [x] **8. Teaching testbed**: one scene per engine concept, in learning order (bodies, shapes
      and collisions, each joint type, structures built from joints, game features, queries,
      stability benchmarks), with labels written in the world and save/restore through serde.
      Full gameplay moved to games: the car became `hill-climb`.
- [x] **9. Performance**: `kinetica-bench` (reproducible scenarios, best of 3 rounds, a
      checksum proving an optimization changes no result) and a `profile` feature timing each
      stage of a step. The broad phase tree is rebalanced with surface area rotations (Box2D
      v3) instead of heights, and traversals no longer allocate; known pairs are filtered by
      a merge with the sorted contact keys. Queries got 34% faster, ragdolls 12%, the
      tumbler 6%; the solver dominates the rest (graph coloring and SIMD would be next).
      A compact solver body copy was tried and dropped: no measurable gain.
- [x] **10. API**: the three games were rewritten as one would like to write them, and the
      engine changed until they compiled: indexing with handles, bodies created with their
      colliders, full sizes, `offset`/`angle` placement, argument-free flags, world-space
      joint anchors, typed joint handles, `rope()`, impact speed in collision events, no-op
      setters that keep bodies asleep, shape queries taking a shape and a position, and
      macroquad colors in `kinetica-macroquad`.
- [x] **11. Internal readability**: `docs/ARCHITECTURE.md` maps the modules and walks through
      a step; `World::step` numbers its seven stages. Joints share their constraint math
      (`Softness::impulse`, `motor_impulse`, `limit_impulse`) instead of repeating the
      formulas, solver passes are a `Pass` enum instead of a boolean, and the shared joint
      methods come from one macro. `contact.rs` became `contact/` (public types, events,
      pair tracking). Bench checksums and a joint probe are bit-identical before and after.
- [x] **12. Games readability**: the window setup, centered messages and box outlines moved to
      `kinetica-macroquad` (`window_conf`, `draw_centered_text`, `DebugRenderer::draw_aabb`)
      instead of being copied in every game. The slingshot names its pop effects and moves to
      the next level with `Game::next_level`, the platformer level is built as values instead
      of filled in, and the car keeps its hubs instead of finding them through its joints.
- [x] **13. Pinball**: a fourth game, using what the others did not: flippers are motorized
      revolute joints with limits, the plunger a prismatic joint pushed by a preloaded spring,
      the table a counter-clockwise chain, the lane gate a one-way collider, and bumpers and
      slingshots kick the ball from collision events. It needed one addition to the engine,
      `ContactPair::other_collider`, to tell which part of a body with many colliders was hit.
      A test plays two minutes of random flipping and checks the ball never leaves the table.
- [x] **14. Performance, kept simple**: contacts and sensor overlaps live in `Vec`s sorted by
      collider pair instead of `BTreeMap`s, so steps walk them contiguously, and solved
      impulses are stored back by walking contacts and constraints together instead of
      looking each one up. The broad phase reports each new pair once instead of sorting out
      duplicates. Bench checksums are unchanged; rain and the tumbler got 9% faster, cloth 7%,
      ragdolls 5%. Graph coloring and SIMD for the solver were set aside on purpose: they would
      make it 2-3x faster but much harder to read, and games rarely need thousands of bodies.
- [x] **15. Accuracy**: measured against textbook results first: sliding on a slope, braking
      by friction, rolling and elastic collisions land within 0.5% of theory, and bounces
      reach 95 to 99% of their ideal height. The weak spot was stiffness: contacts were capped
      at 30 Hz, so extra substeps did not help tall stacks. Contacts and joints now get stiffer
      when substeps are shorter (an eighth of the substep rate, never below 30 Hz), so 8
      substeps hold a column of 18 boxes and 16 a column of 30, and a whipping chain stretches
      half as much. The default 4 substeps behave exactly as before (bench checksums
      unchanged). The testbed panel gained a substep slider. A touching Newton's cradle still
      shares its impact: solving contacts together is what keeps stacks stable.
- [x] **16. Testbed review**: one scene per concept. "Tearing cloth" merged into "Cloth" (a
      curtain you cut holding a heavy bar, next to the hammock), the "Joints" category keeps one
      scene per joint type while the bridge and the ragdolls joined the structures built from
      joints, and the ragdolls scene was reframed so none falls behind the stairs.
- [x] **17. Games from scratch**: the four games were rewritten with one cut-paper look shared
      through `kinetica_macroquad::paper`. `slingshot` (wood, glass and stone towers), `trials`
      (a car over hills, a plank bridge, a seesaw, boxes and a jump, against the clock),
      `platformer` (one-way and moving platforms, an elevator, a box to push, a bridge, coins
      and spikes) and `snip` (cut ropes to drop a sweet in a basket, with bouncy pads and a
      windmill). Pinball and hill-climb were dropped. The engine gained the `WheelJoint` (an
      axle on a sprung, limited suspension axis, with a motor), so a car no longer needs a
      hidden body per wheel; the prismatic and wheel joints share their slide geometry and
      limits. The testbed shows it in a "Wheel joints" scene.
