use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Pyramid",
    category: Category::Benchmarks,
    description: "A 20-box base pyramid: 210 boxes that must stay standing.",
    camera_center: vec2(0.0, 10.0),
    view_height: 24.0,
    build: Build::Static(build),
};

const BASE: usize = 20;
const HALF_SIZE: f32 = 0.5;

fn build(world: &mut World, _labels: &mut Labels) {
    add_ground(world, 40.0);
    for row in 0..BASE {
        let count = BASE - row;
        for column in 0..count {
            let x = (column as f32 - (count - 1) as f32 / 2.0) * 2.0 * HALF_SIZE * 1.05;
            let y = HALF_SIZE + row as f32 * 2.0 * HALF_SIZE;
            let block = world.add_body(BodyDesc::dynamic().position(vec2(x, y)));
            world.add_collider(block, ColliderDesc::rectangle(HALF_SIZE, HALF_SIZE));
        }
    }
}
