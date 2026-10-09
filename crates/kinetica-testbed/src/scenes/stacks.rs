use std::f32::consts::PI;

use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Stacking",
    category: Category::Collisions,
    description: "Structures held only by contacts and friction must stand still, without jitter or drift: tall \
                  columns, a heavy box on light ones (a 20 to 1 mass ratio, hard for solvers), and a stone arch. \
                  Knock them over with the mouse.",
    camera_center: vec2(0.0, 5.5),
    view_height: 13.5,
    build: Build::Static(build),
};

const ARCH_CENTER: Vec2 = vec2(4.5, 0.0);
const ARCH_INNER_RADIUS: f32 = 3.0;
const ARCH_OUTER_RADIUS: f32 = 4.0;
const ARCH_BLOCKS: usize = 11;

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 80.0);

    for (x, width, height, levels) in [(-8.5, 1.0, 1.0, 10), (-6.0, 2.0, 0.5, 16), (-3.5, 0.8, 0.8, 10)] {
        for level in 0..levels {
            let y = height * (level as f32 + 0.5);
            world.add_body(BodyDesc::dynamic().position(vec2(x, y)).collider(ColliderDesc::rectangle(width, height)));
        }
    }
    labels.at(vec2(-6.0, 10.6), "Columns");

    for level in 0..5 {
        let light = ColliderDesc::square(0.5).density(0.25);
        world.add_body(BodyDesc::dynamic().position(vec2(-1.5, 0.25 + 0.5 * level as f32)).collider(light));
    }
    let heavy =
        world.add_body(BodyDesc::dynamic().position(vec2(-1.5, 2.75)).collider(ColliderDesc::square(0.5).density(5.0)));
    labels.above(heavy, "20× heavier\nthan each box\nbelow");

    add_arch(world);
    labels.at(ARCH_CENTER + vec2(0.0, ARCH_OUTER_RADIUS + 1.6), "Arch: no glue, no joints");
}

/// A semicircular arch: each block (voussoir) is the quadrilateral between two radii of the ring.
fn add_arch(world: &mut World) {
    let point_on = |radius: f32, angle: f32| radius * Vec2::from_angle(angle);
    for i in 0..ARCH_BLOCKS {
        let start = PI * i as f32 / ARCH_BLOCKS as f32;
        let end = PI * (i + 1) as f32 / ARCH_BLOCKS as f32;
        let corners = [
            point_on(ARCH_INNER_RADIUS, start),
            point_on(ARCH_OUTER_RADIUS, start),
            point_on(ARCH_OUTER_RADIUS, end),
            point_on(ARCH_INNER_RADIUS, end),
        ];
        // Place the body on the block's center so it rotates naturally around it.
        let center = corners.iter().sum::<Vec2>() / 4.0;
        let local = corners.map(|corner| corner - center);
        let block = ColliderDesc::convex_hull(&local).expect("arch blocks are convex").friction(0.8);
        world.add_body(BodyDesc::dynamic().position(ARCH_CENTER + center).collider(block));
    }
    let load = ColliderDesc::rectangle(1.2, 0.8).density(3.0);
    world.add_body(BodyDesc::dynamic().position(ARCH_CENTER + vec2(0.0, ARCH_OUTER_RADIUS + 0.4)).collider(load));
}
