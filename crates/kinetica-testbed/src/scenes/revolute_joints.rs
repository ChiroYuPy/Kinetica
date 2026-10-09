use std::f32::consts::FRAC_PI_4;

use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Revolute joints",
    category: Category::Joints,
    description: "A revolute joint pins two bodies together at a point, like a hinge or an axle. From left to \
                  right: a free double pendulum (chaotic), a pendulum limited to ±45°, and two arms driven by \
                  the same motor speed; the second motor's torque is too weak to lift its arm.",
    camera_center: vec2(-0.5, 4.5),
    view_height: 14.0,
    build: Build::Static(build),
};

const PIVOT_HEIGHT: f32 = 7.0;
const ARM_LENGTH: f32 = 2.0;
const MOTOR_SPEED: f32 = 1.5;

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 60.0);

    // Double pendulum, released horizontally.
    let pivot = add_pin(world, vec2(-9.0, PIVOT_HEIGHT));
    let mut previous = pivot;
    for i in 0..2 {
        let start = vec2(-9.0 + 1.5 * i as f32, PIVOT_HEIGHT);
        let shape = ColliderDesc::capsule(vec2(-0.75, 0.0), vec2(0.75, 0.0), 0.1);
        let link = world.add_body(BodyDesc::dynamic().position(start + vec2(0.75, 0.0)).collider(shape));
        world.add_joint(RevoluteJointDesc::new(previous, link, start));
        previous = link;
    }
    labels.at(vec2(-8.25, PIVOT_HEIGHT + 1.2), "Free");

    // A pendulum given a push, stopped by its limits. Angles are relative to the start pose.
    let pin = vec2(-3.5, PIVOT_HEIGHT);
    let pivot = add_pin(world, pin);
    let pendulum = world.add_body(with_arm(BodyDesc::dynamic().position(pin).angular_velocity(4.0), -Vec2::Y));
    world.add_joint(RevoluteJointDesc::new(pivot, pendulum, pin).limits(-FRAC_PI_4..=FRAC_PI_4));
    labels.at(pin + vec2(0.0, 1.2), "Limits ±45°");

    // Motors: same target speed, different maximum torques.
    for (x, max_torque, text) in
        [(1.5, 200.0, "Motor 1.5 rad/s\nmax 200 N·m"), (6.5, 15.0, "Motor 1.5 rad/s\nmax 15 N·m: stalls")]
    {
        let pin = vec2(x, PIVOT_HEIGHT - 2.0);
        let pivot = add_pin(world, pin);
        let arm = world.add_body(with_arm(BodyDesc::dynamic().position(pin).angular_damping(1.0), Vec2::X));
        world.add_joint(RevoluteJointDesc::new(pivot, arm, pin).motor(MOTOR_SPEED, max_torque));
        labels.at(pin + vec2(0.0, 3.0), text);
    }
}

/// A fixed pin marking a pivot; a sensor, so the arms swing through it.
fn add_pin(world: &mut World, position: Vec2) -> BodyHandle {
    world.add_body(BodyDesc::fixed().position(position).collider(ColliderDesc::circle(0.12).sensor()))
}

/// Adds a rod from the body origin along `direction`, with a heavy ball at the end.
fn with_arm(body: BodyDesc, direction: Vec2) -> BodyDesc {
    let tip = ARM_LENGTH * direction;
    body.collider(ColliderDesc::capsule(Vec2::ZERO, tip, 0.08))
        .collider(ColliderDesc::circle(0.3).offset(tip).density(5.0))
}
