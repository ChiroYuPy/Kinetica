use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Restitution",
    category: Category::Collisions,
    description: "Each collider has a restitution, its bounciness: 0 does not bounce at all, 1 bounces back to the \
                  height it fell from, forever.",
    camera_center: vec2(0.0, 4.0),
    view_height: 11.0,
    build: Build::Static(build),
};

const RESTITUTIONS: [f32; 6] = [0.0, 0.2, 0.4, 0.6, 0.8, 1.0];

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 40.0);
    for (i, restitution) in RESTITUTIONS.into_iter().enumerate() {
        let shape = ColliderDesc::circle(0.4).restitution(restitution);
        let ball = world.add_body(BodyDesc::dynamic().position(vec2(-4.5 + 1.8 * i as f32, 7.0)).collider(shape));
        labels.above(ball, format!("{restitution}"));
    }
}
