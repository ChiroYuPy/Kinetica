use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Pyramid",
    category: Category::Benchmarks,
    description: "A 20-box base pyramid: 210 boxes that must stay standing.",
    camera_center: vec2(0.0, 10.0),
    view_height: 24.0,
    build: Build::Animated(build),
};

const BASE: usize = 20;
const HALF_SIZE: f32 = 0.5;

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    let mut top = None;
    for row in 0..BASE {
        let count = BASE - row;
        for column in 0..count {
            let x = (column as f32 - (count - 1) as f32 / 2.0) * 2.0 * HALF_SIZE * 1.05;
            let y = HALF_SIZE + row as f32 * 2.0 * HALF_SIZE;
            let block = world.add_body(BodyDesc::dynamic().position(vec2(x, y)));
            world.add_collider(block, ColliderDesc::rectangle(HALF_SIZE, HALF_SIZE));
            top = Some((block, vec2(x, y)));
        }
    }
    labels.at(vec2(0.0, 21.2), "210 boxes resting on each other: errors would pile up toward the top");
    let (top, start) = top.expect("the pyramid has boxes");
    Box::new(TopWatch { top, start })
}

/// Measures how far the top box moved from where it started.
struct TopWatch {
    top: BodyHandle,
    start: Vec2,
}

impl SceneUpdate for TopWatch {
    fn update(&mut self, _world: &mut World, _frame: &Frame) {}

    fn labels(&self, world: &World, _frame: &Frame, labels: &mut Labels) {
        if let Some(body) = world.body(self.top) {
            let drift = 100.0 * body.position().distance(self.start);
            labels.above(self.top, format!("Top box moved {drift:.1} cm"));
        }
    }
}
