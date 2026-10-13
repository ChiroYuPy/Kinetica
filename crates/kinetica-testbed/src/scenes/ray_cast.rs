use kinetica::CollisionLayers;
use kinetica::math::Rot;
use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;

use super::queries::{NORMAL_COLOR, add_targets};
use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Ray cast",
    category: Category::Queries,
    description: "A ray cast finds the first collider on a line, with the point and the normal where it hits. \
                  Aim the laser with the mouse: each bounce is a new ray.",
    camera_center: vec2(0.0, 5.0),
    view_height: 13.0,
    build: Build::Animated(build),
};

const SOLID: u32 = 1 << 0;
const GLASS: u32 = 1 << 1;
const ORIGIN: Vec2 = vec2(-7.5, 3.0);
const LENGTH: f32 = 60.0;
const MAX_BOUNCES: usize = 12;
const LASER_COLOR: Color = Color::new(1.0, 0.3, 0.3, 1.0);
const GLASS_COLOR: Color = Color::new(0.4, 0.75, 1.0, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let bars = add_targets(world, CollisionLayers::new(SOLID, SOLID | GLASS));
    // Panes in their own layer, which the laser's filter leaves out.
    let panes = [(vec2(-5.0, 5.0), 0.2), (vec2(4.0, 5.0), -0.4)].map(|(center, angle)| {
        let pane = ColliderDesc::rectangle(0.16, 2.8).offset(center).angle(angle);
        world.add_collider(bars, pane.layers(CollisionLayers::new(GLASS, SOLID)))
    });
    labels.at(vec2(4.0, 6.7), "Glass: a layer the\nray's filter ignores");
    Box::new(Laser { panes })
}

struct Laser {
    panes: [ColliderHandle; 2],
}

impl SceneUpdate for Laser {
    fn update(&mut self, _world: &mut World, _frame: &Frame) {}

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        for pane in self.panes {
            drawer.draw_collider(world, pane, GLASS_COLOR);
        }
        let filter = QueryFilter::new().layers(SOLID);
        let mut origin = ORIGIN;
        let mut direction = (frame.mouse - ORIGIN).normalize_or(Vec2::X);
        let mut remaining = LENGTH;
        drawer.draw_circle(ORIGIN, 0.2, Rot::IDENTITY, LASER_COLOR);
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
}
