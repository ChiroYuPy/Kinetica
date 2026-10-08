# Kinetica

A 2D rigid-body physics engine for games, written in Rust.

```rust
use kinetica::prelude::*;

let mut world = World::new();
let ball = world.add_body(BodyDesc::dynamic().position(vec2(0.0, 10.0)));
world.add_collider(ball, ColliderDesc::circle(0.5).restitution(0.5));

world.step(1.0 / 60.0);
```

## Workspace

| Crate | Role |
|-------|------|
| [`crates/kinetica`](crates/kinetica) | The engine. No rendering or windowing dependency. |
| [`crates/kinetica-testbed`](crates/kinetica-testbed) | Interactive scenes rendered with macroquad. |

```sh
cargo run -p kinetica-testbed --release   # interactive testbed
cargo test --workspace                     # all tests
cargo clippy --workspace --all-targets     # lints (pedantic)
```

See [docs/ROADMAP.md](docs/ROADMAP.md) for the design decisions and the plan.
