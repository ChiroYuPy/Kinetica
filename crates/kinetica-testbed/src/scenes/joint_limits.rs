use std::f32::consts::FRAC_PI_4;

use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground, add_pin};

pub const SCENE: Scene = Scene {
    name: "Joint limits",
    category: Category::Joints,
    description: "Limits stop a joint at both ends of a range: an angle for a revolute joint, a distance for a \
                  prismatic one. Both pendulums get the same push.",
    camera_center: vec2(0.0, 4.0),
    view_height: 10.0,
    build: Build::Static(build),
};

const PIVOT_HEIGHT: f32 = 6.0;
const ARM_LENGTH: f32 = 2.5;

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 40.0);
    for (x, limited, text) in [(-4.0, false, "No limits"), (4.0, true, "Limits ±45°")] {
        let pin = vec2(x, PIVOT_HEIGHT);
        let pivot = add_pin(world, pin);
        let tip = vec2(0.0, -ARM_LENGTH);
        let pendulum = world.add_body(
            BodyDesc::dynamic()
                .position(pin)
                .angular_velocity(6.0)
                .collider(ColliderDesc::capsule(Vec2::ZERO, tip, 0.08))
                .collider(ColliderDesc::circle(0.3).offset(tip).density(5.0)),
        );
        // Angles are relative to the pose the joint was created in.
        let joint = RevoluteJointDesc::new(pivot, pendulum, pin);
        world.add_joint(if limited { joint.limits(-FRAC_PI_4..=FRAC_PI_4) } else { joint });
        labels.at(pin + vec2(0.0, 0.6), text);
    }
}
