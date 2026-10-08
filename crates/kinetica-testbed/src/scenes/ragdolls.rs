use std::f32::consts::PI;

use kinetica::prelude::*;

use super::{Build, Category, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Ragdolls",
    category: Category::Joints,
    description: "Ragdolls made of ten bodies and nine limited revolute joints tumble down the stairs.",
    camera_center: vec2(0.0, 4.0),
    view_height: 14.0,
    build: Build::Static(build),
};

fn build(world: &mut World) {
    add_ground(world, 40.0);

    // Stairs going down to the right.
    let stairs = world.add_body(BodyDesc::fixed());
    for step in 0..8 {
        let height = 0.4 * (8 - step) as f32;
        let center = vec2(-8.0 + 1.0 * step as f32, height / 2.0);
        world.add_collider(stairs, ColliderDesc::offset_rectangle(0.5, height / 2.0, center, 0.0));
    }

    for (i, x) in [-8.0, -6.5, -5.0].into_iter().enumerate() {
        let floor = 3.2 + 2.5 * i as f32;
        add_ragdoll(world, vec2(x, floor));
    }
}

/// Adds a ragdoll standing with its feet at `feet`.
fn add_ragdoll(world: &mut World, feet: Vec2) {
    let mut part = |center: Vec2, collider: ColliderDesc| {
        let body = world.add_body(BodyDesc::dynamic().position(feet + center).angular_damping(0.5));
        world.add_collider(body, collider.friction(0.6));
        body
    };
    let torso = part(vec2(0.0, 1.3), ColliderDesc::rectangle(0.15, 0.3));
    let head = part(vec2(0.0, 1.82), ColliderDesc::circle(0.15));
    let limbs = [-1.0_f32, 1.0].map(|side| {
        let upper_arm = part(vec2(0.22 * side, 1.4), ColliderDesc::rectangle(0.05, 0.15));
        let lower_arm = part(vec2(0.22 * side, 1.1), ColliderDesc::rectangle(0.045, 0.15));
        let upper_leg = part(vec2(0.09 * side, 0.75), ColliderDesc::rectangle(0.07, 0.25));
        let lower_leg = part(vec2(0.09 * side, 0.25), ColliderDesc::rectangle(0.06, 0.25));
        (side, upper_arm, lower_arm, upper_leg, lower_leg)
    });

    let mut hinge = |a: BodyHandle, b: BodyHandle, point: Vec2, limits: std::ops::RangeInclusive<f32>| {
        world.add_joint(RevoluteJointDesc::at_world_point(world, a, b, feet + point).limits(limits));
    };
    hinge(torso, head, vec2(0.0, 1.65), -0.5..=0.5);
    for (side, upper_arm, lower_arm, upper_leg, lower_leg) in limbs {
        hinge(torso, upper_arm, vec2(0.22 * side, 1.55), -PI * 0.8..=PI * 0.8);
        hinge(upper_arm, lower_arm, vec2(0.22 * side, 1.25), 0.0..=2.4);
        hinge(torso, upper_leg, vec2(0.09 * side, 1.0), -0.6..=1.8);
        hinge(upper_leg, lower_leg, vec2(0.09 * side, 0.5), -2.2..=0.0);
    }
}
