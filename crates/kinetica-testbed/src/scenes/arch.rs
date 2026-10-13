use std::f32::consts::PI;

use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Arch",
    category: Category::Benchmarks,
    description: "A stone arch carrying a load, held by contacts and friction alone: no glue, no joints.",
    camera_center: vec2(0.0, 3.0),
    view_height: 9.0,
    build: Build::Static(build),
};

const INNER_RADIUS: f32 = 3.0;
const OUTER_RADIUS: f32 = 4.0;
const BLOCKS: usize = 11;

fn build(world: &mut World, _labels: &mut Labels) {
    add_ground(world, 40.0);
    // Each block is the quadrilateral between two radii of the ring.
    let point_on = |radius: f32, angle: f32| radius * Vec2::from_angle(angle);
    for i in 0..BLOCKS {
        let start = PI * i as f32 / BLOCKS as f32;
        let end = PI * (i + 1) as f32 / BLOCKS as f32;
        let corners = [
            point_on(INNER_RADIUS, start),
            point_on(OUTER_RADIUS, start),
            point_on(OUTER_RADIUS, end),
            point_on(INNER_RADIUS, end),
        ];
        // Place the body on the block's center so it rotates naturally around it.
        let center = corners.iter().sum::<Vec2>() / 4.0;
        let local = corners.map(|corner| corner - center);
        let block = ColliderDesc::convex_hull(&local).expect("arch blocks are convex").friction(0.8);
        world.add_body(BodyDesc::dynamic().position(center).collider(block));
    }
    let load = ColliderDesc::rectangle(1.2, 0.8).density(3.0);
    world.add_body(BodyDesc::dynamic().position(vec2(0.0, OUTER_RADIUS + 0.4)).collider(load));
}
