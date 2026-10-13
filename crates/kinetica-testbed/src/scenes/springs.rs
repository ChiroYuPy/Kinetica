use kinetica::prelude::*;

use super::{Build, Category, Labels, Scene, add_ground, add_pin};

pub const SCENE: Scene = Scene {
    name: "Springs",
    category: Category::Joints,
    description: "A distance joint with a spring oscillates at its frequency, 1 Hz here. Its damping ratio \
                  decides how fast it settles: 0 never does, 1 settles without bouncing.",
    camera_center: vec2(0.0, 4.5),
    view_height: 11.0,
    build: Build::Static(build),
};

const TOP: f32 = 8.0;
const DAMPING_RATIOS: [f32; 4] = [0.0, 0.1, 0.3, 1.0];

fn build(world: &mut World, labels: &mut Labels) {
    add_ground(world, 40.0);
    // Springs with a 3 m rest length, starting compressed to 1.5 m.
    for (i, damping_ratio) in DAMPING_RATIOS.into_iter().enumerate() {
        let anchor = vec2(-3.75 + 2.5 * i as f32, TOP);
        let weight_position = anchor - vec2(0.0, 1.5);
        let pin = add_pin(world, anchor);
        let weight = world.add_body(BodyDesc::dynamic().position(weight_position).collider(ColliderDesc::square(0.8)));
        world.add_joint(
            DistanceJointDesc::new(pin, weight, anchor, weight_position).length(3.0).spring(1.0, damping_ratio),
        );
        labels.at(anchor + vec2(0.0, 0.4), format!("{damping_ratio}"));
    }
}
