# Kinetica

A 2D rigid-body physics engine for games, written in Rust.

```rust
use kinetica::prelude::*;

let mut world = World::new();
world.add_body(BodyDesc::fixed().collider(ColliderDesc::rectangle(20.0, 1.0)));
let ball = world.add_body(
    BodyDesc::dynamic().position(vec2(0.0, 10.0)).collider(ColliderDesc::circle(0.5).restitution(0.5)),
);

for _ in 0..60 {
    world.step(1.0 / 60.0);
}
println!("The ball is at {}", world[ball].position());
```

Sizes are full sizes (`rectangle(20.0, 1.0)` is 20 m wide), joint anchors are world points, and
indexing the world with a handle reaches the body, collider or joint.

[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) explains how the engine works inside, and
[`docs/ROADMAP.md`](docs/ROADMAP.md) records the design decisions and the history.

## Workspace

| Crate | Role |
|-------|------|
| [`crates/kinetica`](crates/kinetica) | The engine. No rendering or windowing dependency. Optional `serde` and `profile` features. |
| [`crates/kinetica-macroquad`](crates/kinetica-macroquad) | Glue for macroquad games: camera in meters, debug rendering, fixed time step, mouse grab. |
| [`crates/kinetica-bench`](crates/kinetica-bench) | Reproducible performance benchmarks, with the time of each step stage. |
| [`crates/kinetica-testbed`](crates/kinetica-testbed) | One labeled scene per engine concept, from body types to queries. |
| [`games/slingshot`](games/slingshot) | Knock down towers with a slingshot. |
| [`games/platformer`](games/platformer) | Run, jump and collect coins. |
| [`games/hill-climb`](games/hill-climb) | Drive a sprung car over the hills before the fuel runs out. |

```sh
cargo run --release -p kinetica-testbed       # interactive testbed
cargo run --release -p slingshot              # or platformer, hill-climb
cargo run --release -p kinetica-bench         # performance benchmarks
cargo test --workspace --all-features         # all tests
cargo clippy --workspace --all-targets        # lints (pedantic)
```

## Web

Games and the testbed run in the browser through WebAssembly:

```sh
rustup target add wasm32-unknown-unknown
scripts/build-web.sh slingshot Slingshot      # output in target/web/slingshot
python3 -m http.server -d target/web/slingshot 8000
```

See [docs/ROADMAP.md](docs/ROADMAP.md) for the design decisions and the plan.
