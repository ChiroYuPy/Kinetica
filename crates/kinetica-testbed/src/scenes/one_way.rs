use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, Spawner, add_ground, scatter};

pub const SCENE: Scene = Scene {
    name: "One-way platforms",
    category: Category::GameFeatures,
    description: "A one-way collider only stops what comes from one side: balls shot from below go through the \
                  platforms, then land on them.",
    camera_center: vec2(0.0, 5.5),
    view_height: 13.0,
    build: Build::Animated(build),
};

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 30.0);
    let level = world.add_body(BodyDesc::fixed());
    for (center, width) in [(vec2(-3.0, 3.0), 5.0), (vec2(3.0, 5.5), 5.0), (vec2(-2.0, 8.0), 6.0)] {
        world.add_collider(level, ColliderDesc::rectangle(width, 0.2).offset(center).one_way(Vec2::Y));
    }
    // A wall that only stops what comes from its left: drag a ball through it both ways.
    world.add_collider(level, ColliderDesc::rectangle(0.2, 4.0).offset(vec2(8.0, 2.0)).one_way(-Vec2::X));
    labels.at(vec2(8.0, 4.2), "Solid from\nthe left only");

    let mut shots = Spawner::new(0.5, 30);
    Box::new(move |world: &mut World, frame: &Frame| {
        shots.update(world, frame.time, |i| {
            let side = scatter(i);
            BodyDesc::dynamic()
                .position(vec2(-5.0 + 9.0 * side, 0.5))
                .linear_velocity(vec2(1.0 - 2.0 * side, 11.0 + 4.0 * side))
                .collider(ColliderDesc::circle(0.3))
        });
    })
}
