use kinetica::prelude::*;

use super::{Build, Scene};

pub const SCENE: Scene = Scene {
    name: "Pendulums",
    description: "Simple, double and triple pendulums released horizontally. The last two are chaotic.",
    camera_center: vec2(0.0, 0.0),
    view_height: 16.0,
    build: Build::Static(build),
};

const LINK: f32 = 1.5;

fn build(world: &mut World) {
    for (pivot_x, bob_count) in [(-9.0, 1), (-2.0, 2), (4.0, 3)] {
        add_pendulum(world, vec2(pivot_x, 4.0), bob_count);
    }
}

/// A chain of point-like bobs linked by rigid rods, starting horizontal.
fn add_pendulum(world: &mut World, pivot_position: Vec2, bob_count: usize) {
    let pivot = world.add_body(BodyDesc::fixed().position(pivot_position));
    let mut previous = (pivot, pivot_position);
    for i in 1..=bob_count {
        let position = pivot_position + vec2(LINK * i as f32, 0.0);
        let bob = world.add_body(BodyDesc::dynamic().position(position));
        world.add_collider(bob, ColliderDesc::circle(0.2));
        world.add_joint(DistanceJointDesc::between_world_points(world, previous.0, bob, previous.1, position));
        previous = (bob, position);
    }
}
