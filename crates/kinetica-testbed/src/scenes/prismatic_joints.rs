use kinetica::math::Rot;
use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Prismatic joints",
    category: Category::Joints,
    description: "A prismatic joint lets a body slide along an axis without turning, like a piston or a drawer. \
                  Left, a motorized crank pushes a piston through a rod. Middle, an elevator whose motor reverses \
                  every 4 s, stopped by its limits. Right, a free block sliding down a tilted rail to its lower limit.",
    camera_center: vec2(-0.25, 4.0),
    view_height: 14.0,
    build: Build::Animated(build),
};

const ELEVATOR_SPEED: f32 = 1.5;
const ELEVATOR_PERIOD: f32 = 4.0;
const RAIL_CENTER: Vec2 = vec2(6.5, 3.5);
const RAIL_ANGLE: f32 = 0.5;
const RAIL_LENGTH: f32 = 5.0;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 80.0);
    add_crank_slider(world, vec2(-9.5, 3.0));
    labels.at(vec2(-7.0, 5.0), "Crank (revolute motor)\ndrives a piston (prismatic)");

    let elevator = add_elevator(world, vec2(0.0, 0.5));
    labels.at(vec2(0.0, 8.0), "Motor and limits 0 to 5 m");

    add_rail(world);
    labels.at(RAIL_CENTER + vec2(0.0, 2.6), "Free slider with limits");

    Box::new(move |world: &mut World, frame: &Frame| {
        // Up for one period, down for the next.
        let going_up = (frame.time / ELEVATOR_PERIOD).rem_euclid(2.0) < 1.0;
        let speed = if going_up { ELEVATOR_SPEED } else { -ELEVATOR_SPEED };
        world[elevator].set_motor(Some(Motor { speed, max_force: 5000.0 }));
    })
}

/// A crank turned by a motor, a connecting rod, and a piston sliding horizontally.
fn add_crank_slider(world: &mut World, pivot: Vec2) {
    let frame = world.add_body(BodyDesc::fixed().position(pivot));

    let crank_length = 1.0;
    let crank_shape = ColliderDesc::rectangle(crank_length + 0.2, 0.2);
    let crank =
        world.add_body(BodyDesc::dynamic().position(pivot + vec2(crank_length / 2.0, 0.0)).collider(crank_shape));
    world.add_joint(RevoluteJointDesc::new(frame, crank, pivot).motor(2.0, 1000.0));

    let rod_length = 3.0;
    let crank_pin = pivot + vec2(crank_length, 0.0);
    let piston_pin = crank_pin + vec2(rod_length, 0.0);
    let rod_shape = ColliderDesc::rectangle(rod_length, 0.12);
    let rod = world.add_body(BodyDesc::dynamic().position((crank_pin + piston_pin) / 2.0).collider(rod_shape));
    world.add_joint(RevoluteJointDesc::new(crank, rod, crank_pin));

    let piston = world.add_body(BodyDesc::dynamic().position(piston_pin).collider(ColliderDesc::rectangle(0.8, 0.6)));
    world.add_joint(RevoluteJointDesc::new(rod, piston, piston_pin));
    world.add_joint(PrismaticJointDesc::new(frame, piston, piston_pin, Vec2::X));
}

/// A platform on a vertical prismatic joint, carrying a few boxes. Returns the joint.
fn add_elevator(world: &mut World, bottom: Vec2) -> JointHandle<PrismaticJoint> {
    let shaft = world.add_body(BodyDesc::fixed().position(bottom));
    let platform =
        world.add_body(BodyDesc::dynamic().position(bottom).collider(ColliderDesc::rectangle(3.0, 0.2).friction(0.8)));
    let joint = world.add_joint(PrismaticJointDesc::new(shaft, platform, bottom, Vec2::Y).limits(0.0..=5.0));

    for i in 0..3 {
        world.add_body(
            BodyDesc::dynamic().position(bottom + vec2(-0.9 + 0.9 * i as f32, 0.5)).collider(ColliderDesc::square(0.7)),
        );
    }
    joint
}

/// A block on a tilted rail, starting at its upper end.
fn add_rail(world: &mut World) {
    let axis = Rot::from_angle(RAIL_ANGLE).rotate(Vec2::X);
    let rail_shape = ColliderDesc::rectangle(RAIL_LENGTH + 0.8, 0.08).sensor();
    let rail = world.add_body(BodyDesc::fixed().position(RAIL_CENTER).angle(RAIL_ANGLE).collider(rail_shape));

    let start = RAIL_CENTER + (RAIL_LENGTH / 2.0 - 0.4) * axis;
    let block = world
        .add_body(BodyDesc::dynamic().position(start).angle(RAIL_ANGLE).collider(ColliderDesc::rectangle(0.8, 0.5)));
    // Limits are measured along the axis from the starting position.
    let limits = -(RAIL_LENGTH - 0.8)..=0.0;
    world.add_joint(PrismaticJointDesc::new(rail, block, start, axis).limits(limits));
}
