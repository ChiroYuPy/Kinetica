use kinetica::math::Rot;
use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Ray casts",
    category: Category::Queries,
    description: "Move the mouse. The laser on the left casts a ray to the closest hit, then casts again from \
                  there along the reflection. Its filter only accepts the 'solid' layer, so it passes through the \
                  glass panes (blue). The scanner above casts one ray that reports every collider it crosses.",
    camera_center: vec2(0.0, 5.0),
    view_height: 14.0,
    build: Build::Animated(build),
};

const SOLID: u32 = 1 << 0;
const GLASS: u32 = 1 << 1;
const LASER_ORIGIN: Vec2 = vec2(-9.0, 2.0);
const LASER_LENGTH: f32 = 60.0;
const MAX_BOUNCES: usize = 12;
const SCANNER_HEIGHT: f32 = 11.0;

const LASER_COLOR: Color = Color::new(1.0, 0.3, 0.3, 1.0);
const NORMAL_COLOR: Color = Color::new(0.35, 0.85, 0.45, 1.0);
const SCAN_COLOR: Color = Color::new(0.6, 0.6, 0.65, 1.0);
const SCAN_HIT_COLOR: Color = Color::new(1.0, 0.9, 0.3, 1.0);
const GLASS_COLOR: Color = Color::new(0.4, 0.75, 1.0, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 80.0);
    let solid = CollisionLayers::new(SOLID, SOLID | GLASS);
    let glass = CollisionLayers::new(GLASS, SOLID);

    let fixtures = world.add_body(BodyDesc::fixed());
    for (center, length, angle) in [
        (vec2(-3.0, 9.0), 3.0, -0.3),
        (vec2(5.0, 8.0), 2.4, 0.6),
        (vec2(8.5, 3.0), 2.0, 1.3),
        (vec2(1.0, 4.0), 1.6, 0.0),
    ] {
        world.add_collider(fixtures, ColliderDesc::rectangle(length, 0.24).offset(center).angle(angle).layers(solid));
    }
    let mut panes = Vec::new();
    for (center, angle) in [(vec2(-5.0, 4.0), 0.2), (vec2(4.0, 5.5), -0.4)] {
        let pane = ColliderDesc::rectangle(0.16, 2.8).offset(center).angle(angle).layers(glass);
        panes.push(world.add_collider(fixtures, pane));
    }
    labels.at(vec2(-5.0, 6.0), "Glass: another layer");

    for (i, x) in [-6.5_f32, -2.0, 2.5, 6.0].into_iter().enumerate() {
        let shape = if i % 2 == 0 { ColliderDesc::circle(0.7) } else { ColliderDesc::regular_polygon(3 + i, 0.8) };
        world.add_body(BodyDesc::dynamic().position(vec2(x, 1.0)).collider(shape.layers(solid)));
    }
    labels.at(LASER_ORIGIN - vec2(0.0, 0.6), "Laser");

    Box::new(RayDemo { panes })
}

struct RayDemo {
    panes: Vec<ColliderHandle>,
}

impl RayDemo {
    /// The scanner's ray: straight down from above the mouse.
    fn scanner(mouse: Vec2) -> (Vec2, Vec2) {
        (vec2(mouse.x, SCANNER_HEIGHT), vec2(0.0, -SCANNER_HEIGHT - 1.0))
    }
}

impl SceneUpdate for RayDemo {
    fn update(&mut self, _world: &mut World, _frame: &Frame) {}

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        for &pane in &self.panes {
            drawer.draw_collider(world, pane, GLASS_COLOR);
        }

        // The laser: cast, reflect, cast again.
        let filter = QueryFilter::new().layers(SOLID);
        let mut origin = LASER_ORIGIN;
        let mut direction = (frame.mouse - LASER_ORIGIN).normalize_or(Vec2::X);
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

        // The scanner: every hit along one ray, glass included.
        let (start, translation) = Self::scanner(frame.mouse);
        drawer.draw_segment(start, start + translation, SCAN_COLOR);
        for hit in world.cast_ray_all(start, translation, QueryFilter::new()) {
            drawer.draw_point(hit.point, SCAN_HIT_COLOR);
        }
    }

    fn labels(&self, world: &World, frame: &Frame, labels: &mut Labels) {
        let (start, translation) = Self::scanner(frame.mouse);
        let hits = world.cast_ray_all(start, translation, QueryFilter::new()).len();
        labels.at(start + vec2(0.0, 0.2), format!("Scanner: {hits} hits"));
    }
}
