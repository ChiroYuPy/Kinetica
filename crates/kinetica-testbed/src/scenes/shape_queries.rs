use kinetica::math::{Aabb, Rot};
use kinetica::prelude::*;
use kinetica::shape::{Capsule, Circle};
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;
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
const AREA_SIZE: Vec2 = vec2(4.0, 2.4);
const CAST_ORIGIN: Vec2 = vec2(-8.0, 9.0);

const FOUND_COLOR: Color = Color::new(1.0, 0.85, 0.3, 1.0);
const PROBE_COLOR: Color = Color::new(0.45, 0.75, 0.95, 1.0);
const NORMAL_COLOR: Color = Color::new(0.35, 0.85, 0.45, 1.0);
const BOX_COLOR: Color = Color::new(0.9, 0.3, 0.9, 1.0);

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
    add_ground(world, 80.0);
    let obstacles = world.add_body(BodyDesc::fixed());
    for (center, angle) in [(vec2(-3.0, 4.0), 0.3), (vec2(3.0, 6.5), -0.5), (vec2(7.0, 3.0), 0.0)] {
        world.add_collider(obstacles, ColliderDesc::rectangle(2.4, 0.6).offset(center).angle(angle));
    }
    for i in 0..14 {
        let position = vec2(-8.0 + 1.2 * i as f32, 0.6 + 0.3 * (i % 3) as f32);
        let shape = match i % 4 {
            0 => ColliderDesc::circle(0.45),
            1 => ColliderDesc::rectangle(1.0, 0.6),
            2 => ColliderDesc::regular_polygon(3, 0.55),
            _ => ColliderDesc::capsule(vec2(-0.4, 0.0), vec2(0.4, 0.0), 0.2),
        };
        world.add_body(BodyDesc::dynamic().position(position).angle(0.4 * i as f32).collider(shape));
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
                world.colliders_at_point(mouse, filter)
            }
            Query::Overlap => {
                drawer.draw_circle(mouse, PROBE_RADIUS, Rot::IDENTITY, Color { a: 0.6, ..PROBE_COLOR });
                world.colliders_overlapping(Circle::new(PROBE_RADIUS), mouse, filter)
            }
            Query::Area => {
                let area = Aabb::new(mouse - AREA_SIZE / 2.0, mouse + AREA_SIZE / 2.0);
                drawer.draw_aabb(&area, PROBE_COLOR);
                let found = world.colliders_in_aabb(&area, filter);
                for &collider in &found {
                    drawer.draw_aabb(&world[collider].aabb(), BOX_COLOR);
                }
                found
            }
            Query::Cast => {
                let capsule = Capsule::new(vec2(-0.5, 0.0), vec2(0.5, 0.0), 0.3);
                let translation = mouse - CAST_ORIGIN;
                drawer.draw_capsule(CAST_ORIGIN - vec2(0.5, 0.0), CAST_ORIGIN + vec2(0.5, 0.0), 0.3, PROBE_COLOR);
                let Some(hit) = world.cast_shape(capsule, CAST_ORIGIN, translation, filter) else {
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
