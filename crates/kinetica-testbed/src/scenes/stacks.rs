use kinetica::prelude::*;

use super::{Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Stacks",
    description: "Columns of boxes of various sizes and mass ratios, and a ball hitting them.",
    camera_center: vec2(0.0, 6.0),
    view_height: 16.0,
    build,
};

fn build(world: &mut World) {
    add_ground(world, 40.0);

    // Same boxes, wide boxes, and a heavy box on top of light ones.
    for (x, half_width, half_height) in [(-6.0, 0.5, 0.5), (-2.0, 1.0, 0.25), (2.0, 0.4, 0.4)] {
        for level in 0..12 {
            let y = half_height + level as f32 * 2.0 * half_height;
            let block = world.add_body(BodyDesc::dynamic().position(vec2(x, y)));
            world.add_collider(block, ColliderDesc::rectangle(half_width, half_height));
        }
    }
    for level in 0..6 {
        let block = world.add_body(BodyDesc::dynamic().position(vec2(6.0, 0.5 + level as f32)));
        let density = if level == 5 { 20.0 } else { 1.0 };
        world.add_collider(block, ColliderDesc::rectangle(0.5, 0.5).density(density));
    }

    // A fast ball that knocks the right side of the first column after a moment.
    let ball = world.add_body(BodyDesc::dynamic().position(vec2(-16.0, 3.0)).linear_velocity(vec2(12.0, 3.0)));
    world.add_collider(ball, ColliderDesc::circle(0.4).density(4.0));
}
