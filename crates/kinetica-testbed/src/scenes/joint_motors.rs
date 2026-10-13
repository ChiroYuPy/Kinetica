use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground, add_pin};

pub const SCENE: Scene = Scene {
    name: "Joint motors",
    category: Category::Joints,
    description: "A motor drives a joint towards a speed, with a limited force. Both arms want 1.5 rad/s; the \
                  weak motor cannot lift its arm. The elevator's motor reverses.",
    camera_center: vec2(0.0, 4.0),
    view_height: 12.0,
    build: Build::Animated(build),
};

const ARM_LENGTH: f32 = 2.0;
const MOTOR_SPEED: f32 = 1.5;
const ELEVATOR_SPEED: f32 = 1.5;
const ELEVATOR_PERIOD: f32 = 4.0;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);

    for (x, max_torque, text) in [(-7.0, 200.0, "200 N·m"), (-1.5, 15.0, "15 N·m: stalls")] {
        let pin = vec2(x, 4.0);
        let pivot = add_pin(world, pin);
        let tip = vec2(ARM_LENGTH, 0.0);
        let arm = world.add_body(
            BodyDesc::dynamic()
                .position(pin)
                .angular_damping(1.0)
                .collider(ColliderDesc::capsule(Vec2::ZERO, tip, 0.08))
                .collider(ColliderDesc::circle(0.3).offset(tip).density(5.0)),
        );
        world.add_joint(RevoluteJointDesc::new(pivot, arm, pin).motor(MOTOR_SPEED, max_torque));
        labels.at(pin + vec2(0.0, 2.8), text);
    }

    // An elevator: a platform on a vertical prismatic joint, stopped by its limits.
    let bottom = vec2(6.0, 0.5);
    let shaft = world.add_body(BodyDesc::fixed().position(bottom));
    let platform =
        world.add_body(BodyDesc::dynamic().position(bottom).collider(ColliderDesc::rectangle(3.0, 0.2).friction(0.8)));
    let elevator = world.add_joint(PrismaticJointDesc::new(shaft, platform, bottom, Vec2::Y).limits(0.0..=5.0));
    for i in 0..3 {
        world.add_body(
            BodyDesc::dynamic().position(bottom + vec2(-0.9 + 0.9 * i as f32, 0.5)).collider(ColliderDesc::square(0.7)),
        );
    }

    Box::new(move |world: &mut World, frame: &Frame| {
        // Up for one period, down for the next.
        let going_up = (frame.time / ELEVATOR_PERIOD).rem_euclid(2.0) < 1.0;
        let speed = if going_up { ELEVATOR_SPEED } else { -ELEVATOR_SPEED };
        world[elevator].set_motor(Some(Motor { speed, max_force: 5000.0 }));
    })
}
