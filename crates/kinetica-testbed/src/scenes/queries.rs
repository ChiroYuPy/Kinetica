use kinetica::math::{Rot, Transform};
use kinetica::prelude::*;
use kinetica::shape::{Circle, Shape};
use kinetica::{Color, DebugDraw};

use super::{Build, Category, Frame, Scene, SceneUpdate, add_ground};
use crate::render::MacroquadDraw;

pub const SCENE: Scene = Scene {
    name: "Ray and shape casts",
    category: Category::Queries,
    description: "Move the mouse. A ray and a circle are cast from the top left towards it; the circle under \
                  the cursor highlights what it overlaps.",
    camera_center: vec2(0.0, 5.0),
    view_height: 14.0,
    build: Build::Animated(build),
};

const ORIGIN: Vec2 = vec2(-7.5, 10.0);
const CAST_RADIUS: f32 = 0.4;
const PROBE_RADIUS: f32 = 1.0;

const RAY_COLOR: Color = Color::rgb(0.9, 0.9, 0.9);
const MISSED_RAY_COLOR: Color = Color::rgb(0.4, 0.4, 0.45);
const HIT_COLOR: Color = Color::rgb(0.95, 0.3, 0.3);
const OTHER_HIT_COLOR: Color = Color::rgb(0.95, 0.65, 0.65);
const NORMAL_COLOR: Color = Color::rgb(0.35, 0.85, 0.45);
const CAST_COLOR: Color = Color::rgb(0.45, 0.75, 0.95);
const PROBE_COLOR: Color = Color::rgb(0.95, 0.85, 0.3);

fn build(world: &mut World) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    let obstacles = world.add_body(BodyDesc::fixed());
    for (center, angle) in [(vec2(-4.0, 3.0), 0.3), (vec2(2.0, 6.0), -0.5), (vec2(6.0, 2.0), 0.0)] {
        world.add_collider(obstacles, ColliderDesc::offset_rectangle(1.2, 0.4, center, angle));
    }
    for (i, x) in [-6.0_f32, -1.0, 3.0, 8.0].into_iter().enumerate() {
        let body = world.add_body(BodyDesc::dynamic().position(vec2(x, 1.0)));
        let collider = if i % 2 == 0 { ColliderDesc::circle(0.7) } else { ColliderDesc::regular_polygon(3 + i, 0.8) };
        world.add_collider(body, collider);
    }
    Box::new(CastDemo)
}

struct CastDemo;

impl SceneUpdate for CastDemo {
    fn update(&mut self, _world: &mut World, _frame: &Frame) {}

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut MacroquadDraw) {
        let translation = frame.mouse - ORIGIN;
        let filter = QueryFilter::new();

        drawer.draw_circle(ORIGIN, 0.15, Rot::IDENTITY, RAY_COLOR);

        // Ray: every hit, then the closest one with its normal.
        for hit in world.cast_ray_all(ORIGIN, translation, &filter) {
            drawer.draw_point(hit.point, OTHER_HIT_COLOR);
        }
        match world.cast_ray(ORIGIN, translation, &filter) {
            Some(hit) => {
                drawer.draw_segment(ORIGIN, hit.point, RAY_COLOR);
                drawer.draw_segment(hit.point, frame.mouse, MISSED_RAY_COLOR);
                drawer.draw_segment(hit.point, hit.point + 0.5 * hit.normal, NORMAL_COLOR);
                drawer.draw_point(hit.point, HIT_COLOR);
            }
            None => drawer.draw_segment(ORIGIN, frame.mouse, RAY_COLOR),
        }

        // Circle cast along the same path: drawn where it would first touch something.
        let cast_shape = Shape::from(Circle::new(CAST_RADIUS));
        let start = Transform::new(ORIGIN, Rot::IDENTITY);
        if let Some(hit) = world.cast_shape(&cast_shape, &start, translation, &filter) {
            let stop = ORIGIN + hit.fraction * translation;
            drawer.draw_circle(stop, CAST_RADIUS, Rot::IDENTITY, CAST_COLOR);
            drawer.draw_segment(hit.point, hit.point + 0.5 * hit.normal, NORMAL_COLOR);
        }

        // Overlap test under the cursor: highlight the bounding box of what it overlaps.
        let probe = Shape::from(Circle::new(PROBE_RADIUS));
        let probe_transform = Transform::new(frame.mouse, Rot::IDENTITY);
        drawer.draw_circle(frame.mouse, PROBE_RADIUS, Rot::IDENTITY, PROBE_COLOR.with_alpha(0.5));
        for collider in world.colliders_overlapping(&probe, &probe_transform, &filter) {
            let aabb = world.collider(collider).expect("queries return live colliders").aabb();
            let corners = [aabb.min, vec2(aabb.max.x, aabb.min.y), aabb.max, vec2(aabb.min.x, aabb.max.y)];
            for (i, &corner) in corners.iter().enumerate() {
                drawer.draw_segment(corner, corners[(i + 1) % 4], PROBE_COLOR);
            }
        }
    }
}
