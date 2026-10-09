use std::collections::BTreeMap;
use std::f32::consts::TAU;

use kinetica::prelude::*;
use kinetica_macroquad::DebugRenderer;
use macroquad::color::Color;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Contacts and events",
    category: Category::Collisions,
    description: "Touching colliders get contact points (red) with a normal, drawn as long as the force pushing \
                  them apart: the bottom box carries the whole stack. Yellow points are about to touch, so fast \
                  bodies are caught in time. Right, kinematic sweepers cross floating shapes; each shape flashes \
                  when it receives a 'started touching' event.",
    camera_center: vec2(0.5, 4.5),
    view_height: 13.0,
    build: Build::Animated(build),
};

const RING_CENTER: Vec2 = vec2(4.0, 4.8);
/// Drawn length of a contact normal per newton of contact force, in meters.
const LENGTH_PER_NEWTON: f32 = 0.03;
const MIN_NORMAL_LENGTH: f32 = 0.1;
const FLASH_TIME: f32 = 0.3;

const TOUCHING_COLOR: Color = Color::new(0.95, 0.25, 0.25, 1.0);
const SPECULATIVE_COLOR: Color = Color::new(0.95, 0.85, 0.25, 1.0);
const NORMAL_COLOR: Color = Color::new(0.35, 0.85, 0.45, 1.0);
const FLASH_COLOR: Color = Color::new(1.0, 1.0, 0.6, 1.0);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    // A stack on a short ground.
    world.add_body(BodyDesc::fixed().position(vec2(-4.0, -0.25)).collider(ColliderDesc::rectangle(4.0, 0.5)));
    for level in 0..3 {
        world
            .add_body(BodyDesc::dynamic().position(vec2(-4.0, 0.5 + level as f32)).collider(ColliderDesc::square(1.0)));
    }
    labels.at(vec2(-4.0, 4.0), "Contact forces");

    // Two rings of floating shapes, all kinds mixed.
    for (ring, radius, count) in [(0, 2.2, 8), (1, 4.2, 14)] {
        for i in 0..count {
            let angle = TAU * i as f32 / count as f32 + 0.3 * ring as f32;
            let position = RING_CENTER + radius * Vec2::from_angle(angle);
            let shape = match (i + ring) % 4 {
                0 => ColliderDesc::circle(0.4),
                1 => ColliderDesc::rectangle(1.0, 0.56),
                2 => ColliderDesc::regular_polygon(3, 0.5),
                _ => ColliderDesc::regular_polygon(6, 0.4),
            };
            world.add_body(BodyDesc::dynamic().position(position).angle(angle).gravity_scale(0.0).collider(shape));
        }
    }

    // A bar sweeping both rings: its body origin is the pivot, the shape is off-center.
    let bar = ColliderDesc::rectangle(4.4, 0.2).offset(vec2(3.0, 0.0));
    world.add_body(BodyDesc::kinematic().position(RING_CENTER).angular_velocity(0.35).collider(bar));
    // A ball orbiting the other way, through the outer ring.
    let ball = ColliderDesc::circle(0.6).offset(vec2(-4.2, 0.0));
    world.add_body(BodyDesc::kinematic().position(RING_CENTER).angular_velocity(-0.5).collider(ball));

    Box::new(ContactDemo { flashes: BTreeMap::new(), started: 0, stopped: 0, time: 0.0 })
}

struct ContactDemo {
    /// When each body last started touching something.
    flashes: BTreeMap<BodyHandle, f32>,
    started: usize,
    stopped: usize,
    time: f32,
}

impl SceneUpdate for ContactDemo {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        // The events of the previous step.
        for event in world.collision_events() {
            match event {
                CollisionEvent::Started { pair, .. } => {
                    self.started += 1;
                    for body in [pair.body_a, pair.body_b] {
                        self.flashes.insert(body, frame.time);
                    }
                }
                CollisionEvent::Stopped { .. } => self.stopped += 1,
            }
        }
        self.time = frame.time;
    }

    fn draw(&self, world: &World, _frame: &Frame, drawer: &mut DebugRenderer) {
        for (&body, &time) in &self.flashes {
            let age = self.time - time;
            if age < FLASH_TIME && world[body].body_type() == BodyType::Dynamic {
                drawer.draw_body(world, body, Color { a: 1.0 - age / FLASH_TIME, ..FLASH_COLOR });
            }
        }

        // Impulses are those of the last substep.
        let inverse_substep = 60.0 * world.substep_count() as f32;
        for contact in world.contacts() {
            let normal = contact.manifold().normal();
            for point in contact.manifold().points() {
                let position = point.position();
                let force = point.normal_impulse() * inverse_substep;
                let length = (force * LENGTH_PER_NEWTON).max(MIN_NORMAL_LENGTH);
                drawer.draw_segment(position, position + length * normal, NORMAL_COLOR);
                let color = if point.separation() < 0.0 { TOUCHING_COLOR } else { SPECULATIVE_COLOR };
                drawer.draw_point(position, color);
            }
        }
    }

    fn labels(&self, _world: &World, _frame: &Frame, labels: &mut Labels) {
        labels.at(RING_CENTER - vec2(0.0, 5.0), format!("Events: {} started, {} stopped", self.started, self.stopped));
    }
}
