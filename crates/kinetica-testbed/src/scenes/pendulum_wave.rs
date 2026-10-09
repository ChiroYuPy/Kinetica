use std::f32::consts::TAU;

use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Pendulum wave",
    category: Category::Benchmarks,
    description: "Fifteen pendulums whose periods fit 25, 26… 39 swings in 60 s: they drift in and out of sync.",
    camera_center: vec2(0.0, 6.5),
    view_height: 15.0,
    build: Build::Animated(build),
};

const CYCLE: f32 = 60.0;
const FIRST_SWING_COUNT: f32 = 25.0;
const COUNT: usize = 15;
const START_ANGLE: f32 = 0.3;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let gravity = -world.gravity().y;
    for i in 0..COUNT {
        // Simple pendulum: period T = 2π√(L/g), so L = g(T/2π)².
        let period = CYCLE / (FIRST_SWING_COUNT + i as f32);
        let length = gravity * (period / TAU).powi(2);

        let pivot_position = vec2((i as f32 - (COUNT - 1) as f32 / 2.0) * 1.2, 8.0);
        let pivot = world.add_body(BodyDesc::fixed().position(pivot_position));
        let bob_position = pivot_position + length * vec2(START_ANGLE.sin(), -START_ANGLE.cos());
        let bob = world.add_body(BodyDesc::dynamic().position(bob_position).collider(ColliderDesc::circle(0.2)));
        world.add_joint(DistanceJointDesc::new(pivot, bob, pivot_position, bob_position));
    }
    labels.at(vec2(0.0, 8.3), "Length L = g (T / 2 pi)², for 25 to 39 swings in 60 s");
    Box::new(CycleClock)
}

/// Shows where the pendulums are in their 60 s cycle.
struct CycleClock;

impl SceneUpdate for CycleClock {
    fn update(&mut self, _world: &mut World, _frame: &Frame) {}

    fn labels(&self, _world: &World, frame: &Frame, labels: &mut Labels) {
        let time = frame.time % CYCLE;
        labels.at(
            vec2(0.0, 5.5),
            format!("{time:.0} s of {CYCLE:.0} s: back in sync at 0 and 60, opposite halves at 30"),
        );
    }
}
