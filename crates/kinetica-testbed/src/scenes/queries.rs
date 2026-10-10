use kinetica::math::{Aabb, Rot};
use kinetica::prelude::*;
use kinetica::shape::{Capsule, Circle};
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;
use macroquad::input::KeyCode;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Queries",
    category: Category::Queries,
    description: "Ask the world what is where, without simulating anything. Keys 1 to 6 pick a query, the mouse \
                  aims or places it. 1: a laser casting a ray to the closest hit, then again along the reflection; \
                  its filter only accepts the 'solid' layer, so it passes through the glass panes (blue). 2: one \
                  ray reporting every collider it crosses. 3: colliders containing a point. 4: colliders \
                  overlapping a shape. 5: colliders whose bounding box overlaps a box, cheaper but approximate. \
                  6: where a shape moving in a straight line first touches something.",
    camera_center: vec2(0.0, 5.0),
    view_height: 13.0,
    build: Build::Animated(build),
};

const SOLID: u32 = 1 << 0;
const GLASS: u32 = 1 << 1;
const LASER_ORIGIN: Vec2 = vec2(-7.5, 3.0);
const LASER_LENGTH: f32 = 60.0;
const MAX_BOUNCES: usize = 12;
const SCANNER_HEIGHT: f32 = 11.0;
const PROBE_RADIUS: f32 = 1.2;
const AREA_SIZE: Vec2 = vec2(4.0, 2.4);
const CAST_ORIGIN: Vec2 = vec2(-7.0, 10.0);

const LASER_COLOR: Color = Color::new(1.0, 0.3, 0.3, 1.0);
const GLASS_COLOR: Color = Color::new(0.4, 0.75, 1.0, 1.0);
const FOUND_COLOR: Color = Color::new(1.0, 0.85, 0.3, 1.0);
const PROBE_COLOR: Color = Color::new(0.45, 0.75, 0.95, 1.0);
const NORMAL_COLOR: Color = Color::new(0.35, 0.85, 0.45, 1.0);
const BOX_COLOR: Color = Color::new(0.9, 0.3, 0.9, 1.0);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Query {
    Ray,
    RayAll,
    Point,
    Overlap,
    Area,
    Cast,
}

impl Query {
    const KEYS: [(KeyCode, Self); 6] = [
        (KeyCode::Key1, Self::Ray),
        (KeyCode::Key2, Self::RayAll),
        (KeyCode::Key3, Self::Point),
        (KeyCode::Key4, Self::Overlap),
        (KeyCode::Key5, Self::Area),
        (KeyCode::Key6, Self::Cast),
    ];

    fn description(self) -> &'static str {
        match self {
            Self::Ray => "1: cast_ray, bouncing, solid layer only",
            Self::RayAll => "2: cast_ray_all",
            Self::Point => "3: colliders_at_point",
            Self::Overlap => "4: colliders_overlapping (a circle)",
            Self::Area => "5: colliders_in_aabb (bounding boxes, in purple)",
            Self::Cast => "6: cast_shape (a capsule)",
        }
    }
}

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 80.0);
    let solid = CollisionLayers::new(SOLID, SOLID | GLASS);
    let glass = CollisionLayers::new(GLASS, SOLID);

    let fixtures = world.add_body(BodyDesc::fixed());
    for (center, length, angle) in [
        (vec2(-3.0, 8.5), 3.0, -0.3),
        (vec2(5.0, 8.0), 2.4, 0.6),
        (vec2(8.5, 3.5), 2.0, 1.3),
        (vec2(1.0, 4.5), 2.4, 0.0),
    ] {
        world.add_collider(fixtures, ColliderDesc::rectangle(length, 0.3).offset(center).angle(angle).layers(solid));
    }
    let mut panes = Vec::new();
    for (center, angle) in [(vec2(-5.0, 5.0), 0.2), (vec2(4.0, 5.0), -0.4)] {
        let pane = ColliderDesc::rectangle(0.16, 2.8).offset(center).angle(angle).layers(glass);
        panes.push(world.add_collider(fixtures, pane));
    }
    for i in 0..14 {
        let position = vec2(-8.0 + 1.2 * i as f32, 0.6 + 0.3 * (i % 3) as f32);
        let shape = match i % 4 {
            0 => ColliderDesc::circle(0.45),
            1 => ColliderDesc::rectangle(1.0, 0.6),
            2 => ColliderDesc::regular_polygon(3, 0.55),
            _ => ColliderDesc::capsule(vec2(-0.4, 0.0), vec2(0.4, 0.0), 0.2),
        };
        world.add_body(BodyDesc::dynamic().position(position).angle(0.4 * i as f32).collider(shape.layers(solid)));
    }
    Box::new(QueryDemo { query: Query::Ray, panes })
}

struct QueryDemo {
    query: Query,
    panes: Vec<ColliderHandle>,
}

impl QueryDemo {
    /// Casts a ray towards `target`, then again along each reflection.
    fn draw_laser(world: &World, target: Vec2, drawer: &mut DebugRenderer) {
        let filter = QueryFilter::new().layers(SOLID);
        let mut origin = LASER_ORIGIN;
        let mut direction = (target - LASER_ORIGIN).normalize_or(Vec2::X);
        let mut remaining = LASER_LENGTH;
        drawer.draw_circle(LASER_ORIGIN, 0.2, Rot::IDENTITY, LASER_COLOR);
        for _ in 0..=MAX_BOUNCES {
            let Some(hit) = world.cast_ray(origin, remaining * direction, filter) else {
                drawer.draw_segment(origin, origin + remaining * direction, LASER_COLOR);
                break;
            };
            drawer.draw_segment(origin, hit.point, LASER_COLOR);
            drawer.draw_segment(hit.point, hit.point + 0.4 * hit.normal, NORMAL_COLOR);
            remaining *= 1.0 - hit.fraction;
            direction -= 2.0 * direction.dot(hit.normal) * hit.normal;
            // Start just off the surface, or the next ray would hit it again at once.
            origin = hit.point + 1e-3 * hit.normal;
        }
    }

    /// Runs the current query at the mouse and returns the colliders it found.
    fn run(&self, world: &World, mouse: Vec2, drawer: &mut DebugRenderer) -> Vec<ColliderHandle> {
        let filter = QueryFilter::new();
        match self.query {
            Query::Ray => {
                Self::draw_laser(world, mouse, drawer);
                Vec::new()
            }
            Query::RayAll => {
                let (start, translation) = (vec2(mouse.x, SCANNER_HEIGHT), vec2(0.0, -SCANNER_HEIGHT - 1.0));
                drawer.draw_segment(start, start + translation, PROBE_COLOR);
                let hits = world.cast_ray_all(start, translation, filter);
                for hit in &hits {
                    drawer.draw_point(hit.point, NORMAL_COLOR);
                }
                hits.iter().map(|hit| hit.collider).collect()
            }
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
                    return Vec::new();
                };
                let stop = CAST_ORIGIN + hit.fraction * translation;
                drawer.draw_segment(CAST_ORIGIN, stop, PROBE_COLOR);
                drawer.draw_capsule(stop - vec2(0.5, 0.0), stop + vec2(0.5, 0.0), 0.3, FOUND_COLOR);
                drawer.draw_segment(hit.point, hit.point + 0.5 * hit.normal, NORMAL_COLOR);
                vec![hit.collider]
            }
        }
    }
}

impl SceneUpdate for QueryDemo {
    fn update(&mut self, _world: &mut World, frame: &Frame) {
        if let Some(&(_, query)) = Query::KEYS.iter().find(|(key, _)| frame.is_pressed(*key)) {
            self.query = query;
        }
    }

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        for &pane in &self.panes {
            drawer.draw_collider(world, pane, GLASS_COLOR);
        }
        for collider in self.run(world, frame.mouse, drawer) {
            drawer.draw_collider(world, collider, FOUND_COLOR);
        }
    }

    fn labels(&self, _world: &World, _frame: &Frame, labels: &mut Labels) {
        labels.at(vec2(0.0, 11.0), self.query.description());
    }
}
