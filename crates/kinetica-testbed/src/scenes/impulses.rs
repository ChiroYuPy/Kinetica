use kinetica::math::Rot;
use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;
use macroquad::input::KeyCode;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Impulses",
    category: Category::Bodies,
    description: "An impulse changes a velocity at once. E sets off an explosion at the mouse: every body in sight \
                  gets an impulse where the blast hits it.",
    camera_center: vec2(0.0, 5.0),
    view_height: 14.0,
    build: Build::Animated(build),
};

const RADIUS: f32 = 4.0;
/// Velocity change given to a body at the center of an explosion, in m/s.
const SPEED: f32 = 14.0;
/// How long an explosion stays drawn, in seconds.
const FLASH_TIME: f32 = 0.25;
const BLAST_COLOR: Color = Color::new(1.0, 0.85, 0.3, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    let ground = add_ground(world, 24.0);
    for x in [-11.75, 11.75] {
        world.add_collider(ground, ColliderDesc::rectangle(0.5, 24.0).offset(vec2(x, 12.5)));
    }
    for row in 0..6 {
        for column in 0..8 {
            let position = vec2(-3.5 + column as f32, 0.45 + 0.9 * row as f32);
            world.add_body(BodyDesc::dynamic().position(position).collider(ColliderDesc::square(0.9)));
        }
    }
    labels.at(vec2(0.0, -1.4), "E: explosion at the mouse");
    Box::new(Explosions { last: None })
}

struct Explosions {
    /// Center and time of the last explosion.
    last: Option<(Vec2, f32)>,
}

/// Pushes every dynamic body in range that the blast reaches directly.
fn explode(world: &mut World, center: Vec2) {
    let mut pushes = Vec::new();
    for (handle, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
        let offset = body.center_of_mass() - center;
        let distance = offset.length();
        if distance > RADIUS || distance < 1e-3 {
            continue;
        }
        // The first thing the blast meets on its way to the body must be the body itself.
        let Some(hit) = world.cast_ray(center, offset, QueryFilter::new()) else { continue };
        if hit.body != handle {
            continue;
        }
        let impulse = offset / distance * SPEED * (1.0 - distance / RADIUS) * body.mass();
        pushes.push((handle, impulse, hit.point));
    }
    for (body, impulse, point) in pushes {
        // Off-center, so the body also starts spinning.
        world[body].apply_linear_impulse_at_point(impulse, point);
    }
}

impl SceneUpdate for Explosions {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        if frame.is_pressed(KeyCode::E) {
            explode(world, frame.mouse);
            self.last = Some((frame.mouse, frame.time));
        }
    }

    fn draw(&self, _world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        if let Some((center, time)) = self.last {
            let age = (frame.time - time) / FLASH_TIME;
            if age < 1.0 {
                let radius = RADIUS * (0.3 + 0.7 * age);
                drawer.draw_circle(center, radius, Rot::IDENTITY, Color { a: 1.0 - age, ..BLAST_COLOR });
            }
        }
    }
}
