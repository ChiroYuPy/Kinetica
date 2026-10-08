use std::f32::consts::TAU;

use kinetica::prelude::*;

use super::Scene;

pub const SCENE: Scene = Scene {
    name: "Contacts",
    description: "Kinematic sweepers cross floating shapes. Red: overlapping point, yellow: about to touch, \
                  green: normal.",
    camera_center: vec2(0.0, 0.0),
    view_height: 16.0,
    build,
};

fn build(world: &mut World) {
    world.set_gravity(Vec2::ZERO);

    // Two rings of motionless dynamic shapes, all kinds mixed.
    for (ring, radius, count) in [(0, 2.5, 8), (1, 5.0, 14)] {
        for i in 0..count {
            let angle = TAU * i as f32 / count as f32 + 0.3 * ring as f32;
            let body = world.add_body(BodyDesc::dynamic().position(radius * Vec2::from_angle(angle)).angle(angle));
            let collider = match (i + ring) % 4 {
                0 => ColliderDesc::circle(0.45),
                1 => ColliderDesc::rectangle(0.55, 0.3),
                2 => ColliderDesc::regular_polygon(3, 0.55),
                _ => ColliderDesc::regular_polygon(6, 0.45),
            };
            world.add_collider(body, collider);
        }
    }

    // A bar sweeping both rings: its body origin is the pivot, the shape is off-center.
    let bar = world.add_body(BodyDesc::kinematic().angular_velocity(0.35));
    world.add_collider(bar, ColliderDesc::offset_rectangle(2.5, 0.12, vec2(3.5, 0.0), 0.0));

    // A ball orbiting the other way, through the outer ring.
    let orbiter = world.add_body(BodyDesc::kinematic().angular_velocity(-0.5));
    world.add_collider(orbiter, ColliderDesc::new(kinetica::shape::Circle::new(0.7).with_center(vec2(-5.0, 0.0))));
}
