use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground};

pub const SCENE: Scene = Scene {
    name: "Stacking",
    category: Category::Collisions,
    description: "Boxes resting on each other must stand still, without shaking or sinking. More substeps make \
                  contacts stiffer: even taller columns stand.",
    camera_center: vec2(0.0, 5.5),
    view_height: 13.5,
    build: Build::Static(build),
};

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 80.0);
    for (x, width, height, levels) in [(-5.0, 1.0, 1.0, 10), (-2.0, 2.0, 0.5, 16), (1.0, 0.8, 0.8, 10)] {
        for level in 0..levels {
            let y = height * (level as f32 + 0.5);
            world.add_body(BodyDesc::dynamic().position(vec2(x, y)).collider(ColliderDesc::rectangle(width, height)));
        }
    }

    // A large mass ratio is hard for a solver.
    for level in 0..5 {
        let light = ColliderDesc::square(0.5).density(0.25);
        world.add_body(BodyDesc::dynamic().position(vec2(4.5, 0.25 + 0.5 * level as f32)).collider(light));
    }
    let heavy =
        world.add_body(BodyDesc::dynamic().position(vec2(4.5, 2.75)).collider(ColliderDesc::square(0.5).density(5.0)));
    labels.above(heavy, "20× heavier than\neach box below");
}
