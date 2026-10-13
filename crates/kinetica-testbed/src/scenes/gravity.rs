use kinetica::DEFAULT_GRAVITY;
use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;
use macroquad::input::KeyCode;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Gravity",
    category: Category::Bodies,
    description: "The world pulls every dynamic body the same way. The arrow keys point its gravity in a new \
                  direction.",
    camera_center: vec2(0.0, 0.0),
    view_height: 13.0,
    build: Build::Animated(build),
};

/// Inside size of the box.
const WIDTH: f32 = 16.0;
const HEIGHT: f32 = 10.0;
const ARROW_COLOR: Color = Color::new(1.0, 0.9, 0.3, 1.0);

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    // A closed box, so nothing escapes whichever way gravity points.
    let walls = world.add_body(BodyDesc::fixed());
    for (center, width, height) in [
        (vec2(0.0, -HEIGHT / 2.0), WIDTH, 0.5),
        (vec2(0.0, HEIGHT / 2.0), WIDTH, 0.5),
        (vec2(-WIDTH / 2.0, 0.0), 0.5, HEIGHT),
        (vec2(WIDTH / 2.0, 0.0), 0.5, HEIGHT),
    ] {
        world.add_collider(walls, ColliderDesc::rectangle(width, height).offset(center));
    }
    for i in 0..60 {
        let position = vec2(-6.6 + 1.2 * (i % 12) as f32, -3.5 + 1.2 * (i / 12) as f32);
        let shape = if i % 2 == 0 { ColliderDesc::circle(0.4) } else { ColliderDesc::square(0.8) };
        world.add_body(BodyDesc::dynamic().position(position).collider(shape));
    }
    Box::new(Compass)
}

struct Compass;

impl SceneUpdate for Compass {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        let direction = vec2(frame.axis(KeyCode::Left, KeyCode::Right), frame.axis(KeyCode::Down, KeyCode::Up));
        if direction != Vec2::ZERO {
            let gravity = direction.normalize() * DEFAULT_GRAVITY.length();
            if world.gravity() != gravity {
                // Changing gravity wakes every body up.
                world.set_gravity(gravity);
            }
        }
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        let tip = 0.3 * world.gravity();
        drawer.draw_segment(Vec2::ZERO, tip, ARROW_COLOR);
        let back = -0.45 * tip.normalize_or_zero();
        for side in [1.0, -1.0] {
            drawer.draw_segment(tip, tip + back + side * 0.5 * back.perp(), ARROW_COLOR);
        }
    }

    fn labels(&self, world: &World, _frame: &Frame, labels: &mut Labels) {
        let gravity = world.gravity();
        labels.at(vec2(0.0, HEIGHT / 2.0 + 0.6), format!("gravity ({:.1}, {:.1}) m/s²", gravity.x, gravity.y));
    }
}
