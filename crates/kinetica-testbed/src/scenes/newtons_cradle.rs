use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene};

pub const SCENE: Scene = Scene {
    name: "Newton's cradle",
    category: Category::Benchmarks,
    description: "Five bouncy, frictionless balls on strings, 5 cm apart. The first one is released.",
    camera_center: vec2(0.0, 5.5),
    view_height: 11.0,
    build: Build::Static(build),
};

const RADIUS: f32 = 0.5;
const STRING: f32 = 4.0;
const RELEASE_ANGLE: f32 = 1.0;
/// The solver resolves simultaneous contacts together, which spreads momentum over all the
/// balls when they touch. A small gap makes the impacts happen one after the other.
const GAP: f32 = 0.05;

fn build(world: &mut World, _labels: &mut Labels) {
    for i in 0..5 {
        let pivot_position = vec2((i as f32 - 2.0) * (2.0 * RADIUS + GAP), 9.0);
        let pivot = world.add_body(BodyDesc::fixed().position(pivot_position));
        let angle = if i == 0 { -RELEASE_ANGLE } else { 0.0 };
        let ball_position = pivot_position + STRING * vec2(angle.sin(), -angle.cos());
        let ball = world.add_body(BodyDesc::dynamic().position(ball_position));
        world.add_collider(ball, ColliderDesc::circle(RADIUS).restitution(1.0).friction(0.0));
        world.add_joint(DistanceJointDesc::between_world_points(world, pivot, ball, pivot_position, ball_position));
    }
}
