use kinetica::math::Rot;
use kinetica::prelude::*;
use kinetica::{Color, DebugDraw};
use kinetica_macroquad::DebugRenderer;
use macroquad::input::KeyCode;

use super::{Build, Category, Frame, Labels, Scene, SceneUpdate, add_ground};

pub const SCENE: Scene = Scene {
    name: "Forces and impulses",
    category: Category::Bodies,
    description: "A force acts during one step and must be applied again every step: Up fires the rocket \
                  engine, Left/Right apply a torque to steer it. An impulse changes the velocity at once: \
                  press E to set off an explosion at the mouse. It pushes what it can see, at the point it hits, \
                  so bodies also start spinning.",
    camera_center: vec2(0.0, 5.0),
    view_height: 16.0,
    build: Build::Animated(build),
};

const ROCKET_START: Vec2 = vec2(-6.0, 1.2);
/// Engine thrust, in multiples of the rocket's weight.
const THRUST_TO_WEIGHT: f32 = 1.8;
/// Steering angular acceleration, in rad/s².
const STEERING: f32 = 12.0;
const EXPLOSION_RADIUS: f32 = 4.0;
/// Velocity change given to a body at the center of an explosion, in m/s.
const EXPLOSION_SPEED: f32 = 14.0;
/// How long an explosion stays drawn, in seconds.
const FLASH_TIME: f32 = 0.25;

const FLAME_COLOR: Color = Color::rgb(1.0, 0.6, 0.2);
const BLAST_COLOR: Color = Color::rgb(1.0, 0.85, 0.3);

fn build(world: &mut World, labels: &mut Labels) -> Box<dyn SceneUpdate> {
    add_ground(world, 30.0);

    let pad = world.add_body(BodyDesc::fixed().position(ROCKET_START - vec2(0.0, 1.05)));
    world.add_collider(pad, ColliderDesc::rectangle(1.2, 0.15));
    let rocket = world.add_body(BodyDesc::dynamic().position(ROCKET_START).angular_damping(2.0));
    world.add_collider(rocket, ColliderDesc::rectangle(0.3, 0.8));
    let nose = [vec2(-0.3, 0.8), vec2(0.3, 0.8), vec2(0.0, 1.4)];
    world.add_collider(rocket, ColliderDesc::convex_hull(&nose).expect("the nose is a triangle"));
    for side in [-1.0, 1.0] {
        let fin = [vec2(side * 0.3, -0.8), vec2(side * 0.3, -0.2), vec2(side * 0.7, -0.9)];
        world.add_collider(rocket, ColliderDesc::convex_hull(&fin).expect("fins are triangles"));
    }
    labels.at(ROCKET_START + vec2(0.0, 3.2), "Up: engine force\nLeft/Right: torque");

    // Something to blow up.
    for row in 0..5 {
        for column in 0..4 {
            let position = vec2(4.0 + 1.0 * column as f32, 0.45 + 0.9 * row as f32);
            let block = world.add_body(BodyDesc::dynamic().position(position));
            world.add_collider(block, ColliderDesc::rectangle(0.45, 0.45));
        }
    }
    labels.at(vec2(5.5, 6.0), "E: explosion at the mouse");

    Box::new(ForceDemo { rocket, thrusting: false, blast: None })
}

struct ForceDemo {
    rocket: BodyHandle,
    thrusting: bool,
    /// Center and time of the last explosion.
    blast: Option<(Vec2, f32)>,
}

impl ForceDemo {
    /// Forces only last one step: they are applied before every step while the keys are held.
    fn fly(&mut self, world: &mut World, frame: &Frame) {
        let gravity = world.gravity().length();
        let Some(rocket) = world.body_mut(self.rocket) else { return };
        self.thrusting = frame.is_down(KeyCode::Up);
        if self.thrusting {
            let nozzle = rocket.world_point(vec2(0.0, -0.8));
            let thrust = rocket.world_vector(Vec2::Y) * THRUST_TO_WEIGHT * rocket.mass() * gravity;
            rocket.apply_force_at_point(thrust, nozzle);
        }
        let steering = frame.axis(KeyCode::Right, KeyCode::Left);
        if steering != 0.0 {
            let torque = steering * STEERING * rocket.inertia();
            rocket.apply_torque(torque);
        }
    }

    /// Pushes every dynamic body in range that the blast reaches directly.
    fn explode(world: &mut World, center: Vec2) {
        let mut pushes = Vec::new();
        for (handle, body) in world.bodies().filter(|(_, body)| body.body_type() == BodyType::Dynamic) {
            let offset = body.center_of_mass() - center;
            let distance = offset.length();
            if distance > EXPLOSION_RADIUS || distance < 1e-3 {
                continue;
            }
            // The first thing the blast meets on its way to the body must be the body itself.
            let Some(hit) = world.cast_ray(center, offset, &QueryFilter::new()) else { continue };
            if world.collider(hit.collider).map(kinetica::Collider::body) != Some(handle) {
                continue;
            }
            let falloff = 1.0 - distance / EXPLOSION_RADIUS;
            let impulse = offset / distance * EXPLOSION_SPEED * falloff * body.mass();
            pushes.push((handle, impulse, hit.point));
        }
        for (handle, impulse, point) in pushes {
            if let Some(body) = world.body_mut(handle) {
                body.apply_linear_impulse_at_point(impulse, point);
            }
        }
    }
}

impl SceneUpdate for ForceDemo {
    fn update(&mut self, world: &mut World, frame: &Frame) {
        self.fly(world, frame);
        if frame.is_pressed(KeyCode::E) {
            Self::explode(world, frame.mouse);
            self.blast = Some((frame.mouse, frame.time));
        }
    }

    fn draw(&self, world: &World, frame: &Frame, drawer: &mut DebugRenderer) {
        if self.thrusting
            && let Some(rocket) = world.body(self.rocket)
        {
            let nozzle = rocket.world_point(vec2(0.0, -0.8));
            let flicker = 0.8 + 0.2 * (frame.time * 40.0).sin();
            let flame = rocket.world_point(vec2(0.0, -0.8 - 1.0 * flicker));
            for side in [-0.15, 0.15] {
                drawer.draw_segment(rocket.world_point(vec2(side, -0.8)), flame, FLAME_COLOR);
            }
            drawer.draw_segment(nozzle, flame, FLAME_COLOR);
        }
        if let Some((center, time)) = self.blast {
            let age = frame.time - time;
            if age < FLASH_TIME {
                let radius = EXPLOSION_RADIUS * (0.3 + 0.7 * age / FLASH_TIME);
                drawer.draw_circle(center, radius, Rot::IDENTITY, BLAST_COLOR.with_alpha(1.0 - age / FLASH_TIME));
            }
        }
    }
}
