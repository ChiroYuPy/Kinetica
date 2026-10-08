use kinetica::joint::Motor;
use kinetica::prelude::*;

use super::{Build, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Machines",
    description: "Left: a motorised crank drives a piston. Right: an elevator on a prismatic motor reverses every 4 s.",
    camera_center: vec2(0.0, 4.0),
    view_height: 13.0,
    build: Build::Animated(build),
};

const ELEVATOR_SPEED: f32 = 1.5;
const ELEVATOR_PERIOD: f32 = 4.0;

fn build(world: &mut World) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    add_crank_slider(world, vec2(-8.0, 3.0));
    let elevator = add_elevator(world, vec2(5.0, 0.5));

    Box::new(move |world: &mut World, time: f32| {
        // Up for one period, down for the next.
        let going_up = (time / ELEVATOR_PERIOD).rem_euclid(2.0) < 1.0;
        let speed = if going_up { ELEVATOR_SPEED } else { -ELEVATOR_SPEED };
        if let Some(JointKind::Prismatic(slider)) = world.joint_mut(elevator).map(kinetica::Joint::kind_mut) {
            slider.set_motor(Some(Motor { speed, max_force: 5000.0 }));
        }
    })
}

/// A crank turned by a motor, a connecting rod, and a piston sliding horizontally.
fn add_crank_slider(world: &mut World, pivot: Vec2) {
    let frame = world.add_body(BodyDesc::fixed().position(pivot));

    let crank_length = 1.0;
    let crank = world.add_body(BodyDesc::dynamic().position(pivot + vec2(crank_length / 2.0, 0.0)));
    world.add_collider(crank, ColliderDesc::rectangle(crank_length / 2.0 + 0.1, 0.1));
    world.add_joint(RevoluteJointDesc::at_world_point(world, frame, crank, pivot).motor(2.0, 1000.0));

    let rod_length = 3.0;
    let crank_pin = pivot + vec2(crank_length, 0.0);
    let piston_pin = crank_pin + vec2(rod_length, 0.0);
    let rod = world.add_body(BodyDesc::dynamic().position((crank_pin + piston_pin) / 2.0));
    world.add_collider(rod, ColliderDesc::rectangle(rod_length / 2.0, 0.06));
    world.add_joint(RevoluteJointDesc::at_world_point(world, crank, rod, crank_pin));

    let piston = world.add_body(BodyDesc::dynamic().position(piston_pin));
    world.add_collider(piston, ColliderDesc::rectangle(0.4, 0.3));
    world.add_joint(RevoluteJointDesc::at_world_point(world, rod, piston, piston_pin));
    world.add_joint(PrismaticJointDesc::at_world_point(world, frame, piston, piston_pin, Vec2::X));
}

/// A platform on a vertical prismatic joint, carrying a few boxes. Returns the joint.
fn add_elevator(world: &mut World, bottom: Vec2) -> JointHandle {
    let shaft = world.add_body(BodyDesc::fixed().position(bottom));
    let platform = world.add_body(BodyDesc::dynamic().position(bottom));
    world.add_collider(platform, ColliderDesc::rectangle(1.5, 0.1).friction(0.8));
    let joint =
        world.add_joint(PrismaticJointDesc::at_world_point(world, shaft, platform, bottom, Vec2::Y).limits(0.0..=6.0));

    for i in 0..3 {
        let block = world.add_body(BodyDesc::dynamic().position(bottom + vec2(-0.9 + 0.9 * i as f32, 0.5)));
        world.add_collider(block, ColliderDesc::rectangle(0.35, 0.35));
    }
    joint
}
