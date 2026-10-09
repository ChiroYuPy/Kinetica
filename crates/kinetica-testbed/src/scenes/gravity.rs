use kinetica::DEFAULT_GRAVITY;
use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;
use macroquad::input::KeyCode;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Gravity and damping",
    category: Category::Bodies,
    description: "Arrow keys point the world's gravity in a new direction. Each body can scale gravity for \
                  itself (0 floats, negative rises), slow down with damping (like air drag), or have its rotation locked.",
    camera_center: vec2(0.0, 0.0),
    view_height: 13.0,
    build: Build::Animated(build),
};

/// Inside size of the box.
const WIDTH: f32 = 16.0;
const HEIGHT: f32 = 10.0;
const ARROW_COLOR: Color = Color::new(1.0, 0.9, 0.3, 1.0);
const SPECIAL_COLOR: Color = Color::new(0.4, 0.85, 1.0, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
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

    // Ordinary bodies.
    for i in 0..12 {
        let position = vec2(-6.5 + 1.2 * (i % 6) as f32, -3.5 + 1.2 * (i / 6) as f32);
        let shape = if i % 2 == 0 { ColliderDesc::circle(0.4) } else { ColliderDesc::square(0.8) };
        world.add_body(BodyDesc::dynamic().position(position).collider(shape));
    }

    let mut special = Vec::new();
    let mut add_special = |world: &mut World, desc: BodyDesc, collider: ColliderDesc, text: &str| {
        let body = world.add_body(desc.collider(collider));
        labels.above(body, text);
        special.push(body);
    };
    add_special(
        world,
        BodyDesc::dynamic().position(vec2(1.0, 1.0)).gravity_scale(0.0),
        ColliderDesc::circle(0.5),
        "gravity × 0",
    );
    add_special(
        world,
        BodyDesc::dynamic().position(vec2(3.5, -3.5)).gravity_scale(-0.3),
        ColliderDesc::circle(0.5).density(0.2),
        "gravity × -0.3",
    );
    add_special(
        world,
        BodyDesc::dynamic().position(vec2(-2.0, 3.0)).linear_damping(3.0).angular_damping(3.0),
        ColliderDesc::square(1.0),
        "damping 3",
    );
    add_special(
        world,
        BodyDesc::dynamic().position(vec2(5.5, 3.0)).lock_rotation(),
        ColliderDesc::rectangle(1.8, 0.6),
        "rotation locked",
    );

    Box::new(GravityDemo { special })
}

struct GravityDemo {
    special: Vec<BodyHandle>,
}

impl SceneUpdate for GravityDemo {
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
        let back = -0.15 * tip.normalize_or_zero() * 3.0;
        for side in [1.0, -1.0] {
            drawer.draw_segment(tip, tip + back + side * 0.5 * back.perp(), ARROW_COLOR);
        }
        for &body in &self.special {
            drawer.draw_body(world, body, SPECIAL_COLOR);
        }
    }

    fn labels(&self, world: &World, _frame: &Frame, labels: &mut Labels) {
        let gravity = world.gravity();
        labels.at(vec2(0.0, HEIGHT / 2.0 + 1.0), format!("gravity ({:.1}, {:.1}) m/s²", gravity.x, gravity.y));
    }
}
