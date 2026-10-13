use kinetica::prelude::*;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, Spawner, add_ground, scatter};

pub const SCENE: Scene = Scene {
    name: "Shapes",
    category: Category::Collisions,
    description: "The five solid shapes a collider can have. Polygons are convex, with at most eight corners.",
    camera_center: vec2(0.0, 5.5),
    view_height: 15.0,
    build: Build::Animated(build),
};

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let ground = add_ground(world, 24.0);
    // The display on the left, the rain on the right, a low wall between them.
    for (x, height) in [(-11.75, 16.0), (0.0, 3.0), (11.75, 16.0)] {
        world.add_collider(ground, ColliderDesc::rectangle(0.5, height).offset(vec2(x, 0.5 + height / 2.0)));
    }

    let gallery = [
        (ColliderDesc::circle(0.5), "circle"),
        (ColliderDesc::capsule(vec2(-0.5, 0.0), vec2(0.5, 0.0), 0.3), "capsule"),
        (ColliderDesc::rectangle(1.2, 0.8), "rectangle"),
        (ColliderDesc::regular_polygon(5, 0.55), "regular\npolygon"),
        (hull(&[vec2(-0.6, -0.4), vec2(0.7, -0.4), vec2(0.3, 0.5), vec2(-0.3, 0.6)]), "convex\nhull"),
    ];
    for (i, (collider, name)) in gallery.into_iter().enumerate() {
        let bottom = -collider.shape.compute_aabb(&Transform::IDENTITY).min.y;
        let body =
            world.add_body(BodyDesc::dynamic().position(vec2(-10.0 + 2.1 * i as f32, bottom)).collider(collider));
        labels.above(body, name);
    }

    let mut rain = Spawner::new(0.2, 120);
    Box::new(move |world: &mut World, frame: &Frame| {
        rain.update(world, frame.time, |i| {
            BodyDesc::dynamic().position(vec2(1.5 + 9.0 * scatter(i), 14.0)).angle(i as f32).collider(any_shape(i))
        });
    })
}

fn hull(points: &[Vec2]) -> ColliderDesc {
    ColliderDesc::convex_hull(points).expect("the points enclose an area")
}

/// A different shape for each `i`, about half a meter across.
pub fn any_shape(i: usize) -> ColliderDesc {
    let radius = 0.25 + 0.2 * (i as f32 * 0.414_214).fract();
    match i % 5 {
        0 => ColliderDesc::capsule(vec2(-radius, 0.0), vec2(radius, 0.0), 0.5 * radius),
        1 => ColliderDesc::circle(radius),
        2 => ColliderDesc::rectangle(2.0 * radius, 1.4 * radius),
        3 => ColliderDesc::regular_polygon(3 + i % 4, radius),
        _ => hull(&[vec2(-radius, -0.5 * radius), vec2(radius, -0.5 * radius), vec2(0.3 * radius, 0.8 * radius)]),
    }
}
