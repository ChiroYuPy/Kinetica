use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Pyramid",
    category: Category::Benchmarks,
    description: "210 boxes resting on each other: any error of the solver piles up towards the top, whose \
                  drift is measured.",
    camera_center: vec2(0.0, 10.0),
    view_height: 24.0,
    build: Build::Animated(build),
};

const BASE: usize = 20;
const SIZE: f32 = 1.0;

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 80.0);
    let mut top = None;
    for row in 0..BASE {
        let count = BASE - row;
        for column in 0..count {
            let x = (column as f32 - (count - 1) as f32 / 2.0) * SIZE * 1.05;
            let y = SIZE / 2.0 + row as f32 * SIZE;
            let block = world.add_body(BodyDesc::dynamic().position(vec2(x, y)).collider(ColliderDesc::square(SIZE)));
            top = Some((block, vec2(x, y)));
        }
    }
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
        let drift = 100.0 * world[self.top].position().distance(self.start);
        labels.above(self.top, format!("moved {drift:.1} cm"));
    }
}
