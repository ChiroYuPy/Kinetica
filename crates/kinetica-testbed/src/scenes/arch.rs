use std::f32::consts::PI;

use kinetica::prelude::*;

use super::{Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Arch",
    description: "A semicircular stone arch held only by friction and contact forces.",
    camera_center: vec2(0.0, 4.0),
    view_height: 12.0,
    build,
};

const INNER_RADIUS: f32 = 3.5;
const OUTER_RADIUS: f32 = 4.5;
const BLOCK_COUNT: usize = 11;

fn build(world: &mut World) {
    add_ground(world, 20.0);

    // Each block (voussoir) is the quadrilateral between two radii of the ring.
    let point_on = |radius: f32, angle: f32| radius * Vec2::from_angle(angle);
    for i in 0..BLOCK_COUNT {
        let start = PI * i as f32 / BLOCK_COUNT as f32;
        let end = PI * (i + 1) as f32 / BLOCK_COUNT as f32;
        let corners = [
            point_on(INNER_RADIUS, start),
            point_on(OUTER_RADIUS, start),
            point_on(OUTER_RADIUS, end),
            point_on(INNER_RADIUS, end),
        ];
        // Place the body on the block's center so it rotates naturally around it.
        let center = corners.iter().sum::<Vec2>() / 4.0;
        let local = corners.map(|corner| corner - center);
        let block = world.add_body(BodyDesc::dynamic().position(center));
        world.add_collider(block, ColliderDesc::convex_hull(&local).expect("arch blocks are convex").friction(0.8));
    }

    // Something heavy on top to prove the point.
    let weight = world.add_body(BodyDesc::dynamic().position(vec2(0.0, OUTER_RADIUS + 0.6)));
    world.add_collider(weight, ColliderDesc::rectangle(0.6, 0.4).density(3.0));
}
