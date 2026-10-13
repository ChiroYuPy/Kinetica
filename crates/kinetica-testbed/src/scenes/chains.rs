use std::f32::consts::TAU;

use kinetica::prelude::*;

use super::shapes::any_shape;
use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, Spawner, scatter};

pub const SCENE: Scene = Scene {
    name: "Chains and segments",
    category: Category::Collisions,
    description: "Shapes for levels. A chain is a line of points, solid on one side only and smooth to slide on; a \
                  segment is a thin wall, solid on both sides.",
    camera_center: vec2(0.0, 5.5),
    view_height: 15.0,
    build: Build::Animated(build),
};

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let level = world.add_body(BodyDesc::fixed());

    // Wavy ground between two walls, listed left to right so it faces up.
    let mut ground = vec![vec2(-11.0, 14.0)];
    ground.extend((0..=44).map(|i| {
        let x = -11.0 + 0.5 * i as f32;
        vec2(x, 0.6 * (0.7 * x).sin())
    }));
    ground.push(vec2(11.0, 14.0));
    world.add_chain(level, ChainDesc::new(ground).friction(0.7));
    labels.at(vec2(0.0, -1.6), "Chain: solid on the side of its ticks");

    // A rock: listed clockwise, so it is solid from the outside.
    let rock: Vec<Vec2> = (0..9)
        .map(|i| {
            let angle = -TAU * i as f32 / 9.0;
            vec2(4.5, 5.5) + (1.3 + 0.4 * (3.0 * angle).cos()) * Vec2::from_angle(angle)
        })
        .collect();
    world.add_chain(level, ChainDesc::new(rock).closed());
    labels.at(vec2(4.5, 5.2), "Closed\nchain");

    world.add_collider(level, ColliderDesc::segment(vec2(-7.5, 7.0), vec2(-3.0, 5.0)));
    labels.at(vec2(-5.5, 4.6), "Segment");

    let mut rain = Spawner::new(0.2, 120);
    Box::new(move |world: &mut World, frame: &Frame| {
        rain.update(world, frame.time, |i| {
            BodyDesc::dynamic().position(vec2(-9.0 + 18.0 * scatter(i), 13.0)).angle(i as f32).collider(any_shape(i))
        });
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nothing_falls_through_the_level() {
        let mut world = World::new();
        let mut scene = build(&mut world, &mut Labels::default());
        for step in 0..30 * 60 {
            let frame = Frame { time: step as f32 / 60.0, ..Frame::default() };
            scene.update(&mut world, &frame);
            world.step(1.0 / 60.0);
        }
        for (_, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
            let position = body.position();
            assert!(position.x.abs() < 11.0 && position.y > -0.8, "a body escaped to {position}");
        }
    }
}
