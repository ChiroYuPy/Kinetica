use std::collections::BTreeMap;
use std::f32::consts::TAU;

use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Contact events",
    category: Category::Collisions,
    description: "The world reports when two colliders start and stop touching. Here, a shape flashes when it gets \
                  a 'started' event.",
    camera_center: vec2(0.0, 0.0),
    view_height: 12.0,
    build: Build::Animated(build),
};

const FLASH_TIME: f32 = 0.3;
const FLASH_COLOR: Color = Color::new(1.0, 1.0, 0.6, 1.0);

fn build(world: &mut World, _labels: &mut Labels) -> Box<dyn SceneUpdate> {
    // Two rings of floating shapes.
    for (ring, radius, count) in [(0, 2.2, 8), (1, 4.2, 14)] {
        for i in 0..count {
            let angle = TAU * i as f32 / count as f32 + 0.3 * ring as f32;
            let shape = match (i + ring) % 3 {
                0 => ColliderDesc::circle(0.4),
                1 => ColliderDesc::rectangle(1.0, 0.56),
                _ => ColliderDesc::regular_polygon(3, 0.5),
            };
            let body = BodyDesc::dynamic().position(radius * Vec2::from_angle(angle)).angle(angle).gravity_scale(0.0);
            world.add_body(body.collider(shape));
        }
    }
    // A bar sweeping both rings: its body origin is the pivot, the shape is off-center.
    let bar = ColliderDesc::rectangle(4.4, 0.2).offset(vec2(3.0, 0.0));
    world.add_body(BodyDesc::kinematic().angular_velocity(0.35).collider(bar));

    Box::new(Flashes { last_touch: BTreeMap::new(), started: 0, stopped: 0 })
}

struct Flashes {
    /// When each body last started touching something.
    last_touch: BTreeMap<BodyHandle, f32>,
    started: usize,
    stopped: usize,
}

impl SceneUpdate for Flashes {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        // The events of the previous step.
        for event in world.collision_events() {
            match event {
                CollisionEvent::Started { pair, .. } => {
                    self.started += 1;
                    for body in [pair.body_a, pair.body_b] {
                        self.last_touch.insert(body, frame.time);
                    }
                }
                CollisionEvent::Stopped { .. } => self.stopped += 1,
            }
        }
    }

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        for (&body, &time) in &self.last_touch {
            let age = (frame.time - time) / FLASH_TIME;
            if age < 1.0 && world.get_body(body).is_some_and(|body| body.body_type() == BodyType::Dynamic) {
                drawer.draw_body(world, body, Color { a: 1.0 - age, ..FLASH_COLOR });
            }
        }
    }

    fn labels(&self, _world: &World, _frame: &Frame, labels: &mut Labels) {
        labels.at(vec2(0.0, -5.6), format!("{} started, {} stopped", self.started, self.stopped));
    }
}
