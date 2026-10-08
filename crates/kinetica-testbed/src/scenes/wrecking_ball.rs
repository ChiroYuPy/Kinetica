use kinetica::prelude::*;

use super::{Build, Category, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Wrecking ball",
    category: Category::Joints,
    description: "A heavy ball on a chain of revolute joints swings into a tower of boxes.",
    camera_center: vec2(0.0, 6.0),
    view_height: 16.0,
    build: Build::Static(build),
};

const LINK_HALF_LENGTH: f32 = 0.25;
const LINK_COUNT: usize = 14;
const BALL_RADIUS: f32 = 0.8;

fn build(world: &mut World) {
    add_ground(world, 40.0);

    // Two columns of boxes in the way of the swing.
    for column in 0..2 {
        for level in 0..8 {
            let position = vec2(1.5 + 1.0 * column as f32, 0.5 + level as f32);
            let block = world.add_body(BodyDesc::dynamic().position(position));
            world.add_collider(block, ColliderDesc::rectangle(0.45, 0.5));
        }
    }

    // The chain starts horizontal, to the left of its anchor.
    let anchor_position = vec2(0.0, 13.0);
    let mut previous = world.add_body(BodyDesc::fixed().position(anchor_position));
    let mut joint_position = anchor_position;
    for _ in 0..LINK_COUNT {
        let link = world.add_body(BodyDesc::dynamic().position(joint_position - vec2(LINK_HALF_LENGTH, 0.0)));
        world.add_collider(link, ColliderDesc::rectangle(LINK_HALF_LENGTH, 0.06).density(2.0));
        world.add_joint(RevoluteJointDesc::at_world_point(world, previous, link, joint_position));
        previous = link;
        joint_position -= vec2(2.0 * LINK_HALF_LENGTH, 0.0);
    }
    let ball = world.add_body(BodyDesc::dynamic().position(joint_position - vec2(BALL_RADIUS, 0.0)));
    world.add_collider(ball, ColliderDesc::circle(BALL_RADIUS).density(8.0));
    world.add_joint(RevoluteJointDesc::at_world_point(world, previous, ball, joint_position));
}
