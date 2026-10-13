use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Wheel joint",
    category: Category::Joints,
    description: "An axle on a sprung suspension, with a motor: one joint per wheel. The soft suspension soaks up \
                  the bumps, the stiff one shakes.",
    camera_center: vec2(0.0, 2.0),
    view_height: 7.0,
    build: Build::Animated(build),
};

const DRIVE_SPEED: f32 = 6.0;
const DRIVE_PERIOD: f32 = 4.0;
/// The walls at both ends.
const HALF_WIDTH: f32 = 9.5;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    // Bumpy ground between two walls, listed left to right so it is solid below.
    let ground = world.add_body(BodyDesc::fixed());
    let mut outline = vec![vec2(-HALF_WIDTH, 3.0)];
    outline.extend((-19..=19).map(|i| {
        let x = 0.5 * i as f32;
        vec2(x, if x.abs() < 5.0 { 0.15 * (1.0 - (2.0 * x).cos()) } else { 0.0 })
    }));
    outline.push(vec2(HALF_WIDTH, 3.0));
    world.add_chain(ground, ChainDesc::new(outline));

    let soft = add_cart(world, vec2(-7.0, 1.0), 2.0);
    let stiff = add_cart(world, vec2(-2.0, 1.0), 8.0);
    labels.above(soft[0].0, "Soft: 2 Hz");
    labels.above(stiff[0].0, "Stiff: 8 Hz");

    let motors: Vec<JointHandle<WheelJoint>> = soft.iter().chain(&stiff).map(|&(_, motor)| motor).collect();
    Box::new(move |world: &mut World, frame: &Frame| {
        // Right for one period, left for the next.
        let forward = (frame.time / DRIVE_PERIOD).rem_euclid(2.0) < 1.0;
        let speed = if forward { -DRIVE_SPEED } else { DRIVE_SPEED };
        for &motor in &motors {
            world[motor].set_motor(Some(Motor { speed, max_force: 40.0 }));
        }
    })
}

/// A cart whose two wheels hang on springs of `hertz`; returns, for each wheel, the cart and the
/// wheel joint.
fn add_cart(world: &mut World, position: Vec2, hertz: f32) -> [(BodyHandle, JointHandle<WheelJoint>); 2] {
    let cart = world.add_body(BodyDesc::dynamic().position(position).collider(ColliderDesc::rectangle(3.0, 0.5)));
    [-1.1, 1.1].map(|x| {
        let axle = position + vec2(x, -0.6);
        let tire = ColliderDesc::circle(0.35).friction(1.0);
        let wheel = world.add_body(BodyDesc::dynamic().position(axle).collider(tire));
        let joint = WheelJointDesc::new(cart, wheel, axle, Vec2::Y).spring(hertz, 0.5).limits(-0.3..=0.15);
        (cart, world.add_joint(joint))
    })
}
