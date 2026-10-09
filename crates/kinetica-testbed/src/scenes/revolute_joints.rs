use std::f32::consts::FRAC_PI_4;

use kinetica::prelude::*;
use kinetica::shape::Circle;

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
    add_ground(world, 30.0);

    // Double pendulum, released horizontally.
    let pivot = add_pin(world, vec2(-9.0, PIVOT_HEIGHT));
    let mut previous = pivot;
    for i in 0..2 {
        let start = vec2(-9.0 + 1.5 * i as f32, PIVOT_HEIGHT);
        let link = world.add_body(BodyDesc::dynamic().position(start + vec2(0.75, 0.0)));
        world.add_collider(link, ColliderDesc::capsule(vec2(-0.75, 0.0), vec2(0.75, 0.0), 0.1));
        world.add_joint(RevoluteJointDesc::at_world_point(world, previous, link, start));
        previous = link;
    }
    labels.at(vec2(-8.25, PIVOT_HEIGHT + 1.2), "Free");

    // A pendulum given a push, stopped by its limits. Angles are relative to the start pose.
    let pin = vec2(-3.5, PIVOT_HEIGHT);
    let pivot = add_pin(world, pin);
    let pendulum = world.add_body(BodyDesc::dynamic().position(pin).angular_velocity(4.0));
    add_arm(world, pendulum, -Vec2::Y);
    world.add_joint(RevoluteJointDesc::at_world_point(world, pivot, pendulum, pin).limits(-FRAC_PI_4..=FRAC_PI_4));
    labels.at(pin + vec2(0.0, 1.2), "Limits ±45°");

    // Motors: same target speed, different maximum torques.
    for (x, max_torque, text) in
        [(1.5, 200.0, "Motor 1.5 rad/s\nmax 200 N·m"), (6.5, 15.0, "Motor 1.5 rad/s\nmax 15 N·m: stalls")]
    {
        let pin = vec2(x, PIVOT_HEIGHT - 2.0);
        let pivot = add_pin(world, pin);
        let arm = world.add_body(BodyDesc::dynamic().position(pin).angular_damping(1.0));
        add_arm(world, arm, Vec2::X);
        world.add_joint(RevoluteJointDesc::at_world_point(world, pivot, arm, pin).motor(MOTOR_SPEED, max_torque));
        labels.at(pin + vec2(0.0, 3.0), text);
    }
}

/// A fixed pin marking a pivot; a sensor, so the arms swing through it.
fn add_pin(world: &mut World, position: Vec2) -> BodyHandle {
    let pin = world.add_body(BodyDesc::fixed().position(position));
    world.add_collider(pin, ColliderDesc::circle(0.12).sensor(true));
    pin
}

/// A rod from the body origin along `direction`, with a heavy ball at the end.
fn add_arm(world: &mut World, body: BodyHandle, direction: Vec2) {
    let tip = ARM_LENGTH * direction;
    world.add_collider(body, ColliderDesc::capsule(Vec2::ZERO, tip, 0.08));
    world.add_collider(body, ColliderDesc::new(Circle::new(0.3).with_center(tip)).density(5.0));
}
