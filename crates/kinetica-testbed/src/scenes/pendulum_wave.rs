use std::f32::consts::TAU;

use kinetica::prelude::*;

use super::{Build, Scene};

pub const SCENE: Scene = Scene {
    name: "Pendulum wave",
    description: "Fifteen pendulums whose periods fit 25, 26… 39 swings in 60 s: they drift in and out of sync.",
    camera_center: vec2(0.0, 5.0),
    view_height: 9.0,
    build: Build::Static(build),
};

const CYCLE: f32 = 60.0;
const FIRST_SWING_COUNT: f32 = 25.0;
const COUNT: usize = 15;
const START_ANGLE: f32 = 0.3;

fn build(world: &mut World) {
    let gravity = -world.gravity().y;
    for i in 0..COUNT {
        // Simple pendulum: period T = 2π√(L/g), so L = g(T/2π)².
        let period = CYCLE / (FIRST_SWING_COUNT + i as f32);
        let length = gravity * (period / TAU).powi(2);

        let pivot_position = vec2((i as f32 - (COUNT - 1) as f32 / 2.0) * 1.2, 8.0);
        let pivot = world.add_body(BodyDesc::fixed().position(pivot_position));
        let bob_position = pivot_position + length * vec2(START_ANGLE.sin(), -START_ANGLE.cos());
        let bob = world.add_body(BodyDesc::dynamic().position(bob_position));
        world.add_collider(bob, ColliderDesc::circle(0.2));
        world.add_joint(DistanceJointDesc::between_world_points(world, pivot, bob, pivot_position, bob_position));
    }
}
