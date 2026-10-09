use kinetica::math::{Aabb, Rot, Transform};
use kinetica::prelude::*;
use kinetica::shape::{Capsule, Circle, Shape};
use kinetica::{Color, DebugDraw};
use kinetica_macroquad::DebugRenderer;
use macroquad::input::KeyCode;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Shape queries",
    category: Category::Queries,
    description: "Ask the world what is where, without simulating anything. Keys 1 to 4 pick a query, the mouse \
                  places it. 1: colliders containing a point. 2: colliders overlapping a shape. 3: colliders whose \
                  bounding box overlaps a box, cheaper but approximate. 4: where a shape moving in a straight line \
                  first touches something.",
    camera_center: vec2(0.0, 4.5),
    view_height: 12.0,
    build: Build::Animated(build),
};

const PROBE_RADIUS: f32 = 1.2;
const AREA_HALF_SIZE: Vec2 = vec2(2.0, 1.2);
const CAST_ORIGIN: Vec2 = vec2(-8.0, 9.0);

const FOUND_COLOR: Color = Color::rgb(1.0, 0.85, 0.3);
const PROBE_COLOR: Color = Color::rgb(0.45, 0.75, 0.95);
const NORMAL_COLOR: Color = Color::rgb(0.35, 0.85, 0.45);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Query {
    Point,
    Overlap,
    Area,
    Cast,
}

impl Query {
    fn description(self) -> &'static str {
        match self {
            Self::Point => "1: colliders_at_point",
            Self::Overlap => "2: colliders_overlapping (a circle)",
            Self::Area => "3: colliders_in_aabb (bounding boxes, in purple)",
            Self::Cast => "4: cast_shape (a capsule)",
        }
    }
}

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 40.0);
    let obstacles = world.add_body(BodyDesc::fixed());
    for (center, angle) in [(vec2(-3.0, 4.0), 0.3), (vec2(3.0, 6.5), -0.5), (vec2(7.0, 3.0), 0.0)] {
        world.add_collider(obstacles, ColliderDesc::offset_rectangle(1.2, 0.3, center, angle));
    }
    for i in 0..14 {
        let position = vec2(-8.0 + 1.2 * i as f32, 0.6 + 0.3 * (i % 3) as f32);
        let body = world.add_body(BodyDesc::dynamic().position(position).angle(0.4 * i as f32));
        let collider = match i % 4 {
            0 => ColliderDesc::circle(0.45),
            1 => ColliderDesc::rectangle(0.5, 0.3),
            2 => ColliderDesc::regular_polygon(3, 0.55),
            _ => ColliderDesc::capsule(vec2(-0.4, 0.0), vec2(0.4, 0.0), 0.2),
        };
        world.add_collider(body, collider);
    }
    Box::new(QueryDemo { query: Query::Point })
}

struct QueryDemo {
    query: Query,
}

impl SceneUpdate for QueryDemo {
    fn update(&mut self, _world: &mut World, frame: &Frame) {
        for (key, query) in [
            (KeyCode::Key1, Query::Point),
            (KeyCode::Key2, Query::Overlap),
            (KeyCode::Key3, Query::Area),
            (KeyCode::Key4, Query::Cast),
        ] {
            if frame.is_pressed(key) {
                self.query = query;
            }
        }
    }

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        let filter = QueryFilter::new();
        let mouse = frame.mouse;
        let found = match self.query {
            Query::Point => {
                drawer.draw_point(mouse, PROBE_COLOR);
                world.colliders_at_point(mouse, &filter)
            }
            Query::Overlap => {
                let probe = Shape::from(Circle::new(PROBE_RADIUS));
                drawer.draw_circle(mouse, PROBE_RADIUS, Rot::IDENTITY, PROBE_COLOR.with_alpha(0.6));
                world.colliders_overlapping(&probe, &Transform::new(mouse, Rot::IDENTITY), &filter)
            }
            Query::Area => {
                let area = Aabb::new(mouse - AREA_HALF_SIZE, mouse + AREA_HALF_SIZE);
                draw_aabb(drawer, &area, PROBE_COLOR);
                let found = world.colliders_in_aabb(&area, &filter);
                for &collider in &found {
                    if let Some(collider) = world.collider(collider) {
                        draw_aabb(drawer, &collider.aabb(), Color::rgb(0.9, 0.3, 0.9));
                    }
                }
                found
            }
            Query::Cast => {
                let capsule = Capsule::new(vec2(-0.5, 0.0), vec2(0.5, 0.0), 0.3);
                let shape = Shape::from(capsule);
                let start = Transform::new(CAST_ORIGIN, Rot::IDENTITY);
                let translation = mouse - CAST_ORIGIN;
                drawer.draw_capsule(CAST_ORIGIN - vec2(0.5, 0.0), CAST_ORIGIN + vec2(0.5, 0.0), 0.3, PROBE_COLOR);
                let Some(hit) = world.cast_shape(&shape, &start, translation, &filter) else {
                    drawer.draw_segment(CAST_ORIGIN, mouse, PROBE_COLOR);
                    return;
                };
                let stop = CAST_ORIGIN + hit.fraction * translation;
                drawer.draw_segment(CAST_ORIGIN, stop, PROBE_COLOR);
                drawer.draw_capsule(stop - vec2(0.5, 0.0), stop + vec2(0.5, 0.0), 0.3, FOUND_COLOR);
                drawer.draw_segment(hit.point, hit.point + 0.5 * hit.normal, NORMAL_COLOR);
                drawer.draw_point(hit.point, NORMAL_COLOR);
                vec![hit.collider]
            }
        };
        for collider in found {
            drawer.draw_collider(world, collider, FOUND_COLOR);
        }
    }

    fn labels(&self, _world: &World, _frame: &Frame, labels: &mut Labels) {
        labels.at(vec2(0.0, 10.0), self.query.description());
    }
}

fn draw_aabb(drawer: &mut DebugRenderer, aabb: &Aabb, color: Color) {
    let corners = [aabb.min, vec2(aabb.max.x, aabb.min.y), aabb.max, vec2(aabb.min.x, aabb.max.y)];
    for (i, &corner) in corners.iter().enumerate() {
        drawer.draw_segment(corner, corners[(i + 1) % 4], color);
    }
}
