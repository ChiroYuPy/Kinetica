use kinetica::joint::Motor;
use kinetica::prelude::*;
use macroquad::input::{KeyCode, is_key_down};

use super::{Build, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Car",
    description: "Up/Down to drive. Wheels: revolute motors on hubs sliding on prismatic joints held by springs.",
    camera_center: vec2(0.0, 2.0),
    view_height: 12.0,
    build: Build::Animated(build),
};

const WHEEL_RADIUS: f32 = 0.4;
const DRIVE_SPEED: f32 = 25.0;
const DRIVE_TORQUE: f32 = 60.0;
/// Small torque holding the wheels when no key is pressed, like rolling resistance.
const IDLE_TORQUE: f32 = 2.0;

fn build(world: &mut World) -> Box<dyn SceneUpdate> {
    add_terrain(world);
    Box::new(add_car(world, vec2(0.0, 1.5)))
}

/// Rolling hills made of rectangles laid end to end.
fn add_terrain(world: &mut World) {
    let ground = world.add_body(BodyDesc::fixed());
    let heights = [
        0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 1.8, 2.0, 1.5, 0.5, 0.0, -0.3, 0.0, 0.8, 0.8, 0.0, 0.0, 1.0, 2.5, 4.0, 4.0, 3.0,
        1.0, 0.0, 0.0, 0.0,
    ];
    let start_x = -10.0;
    let segment_width = 2.0;
    let points: Vec<Vec2> =
        heights.iter().enumerate().map(|(i, &height)| vec2(start_x + segment_width * i as f32, height)).collect();
    for pair in points.windows(2) {
        let (start, end) = (pair[0], pair[1]);
        let direction = end - start;
        let thickness = 0.25;
        // Sink the rectangle by its half thickness so its top edge follows the profile.
        let normal = direction.perp().normalize();
        let center = (start + end) / 2.0 - thickness * normal;
        let angle = direction.y.atan2(direction.x);
        // Slightly longer than the segment so neighbours overlap and wheels do not catch on seams.
        world.add_collider(
            ground,
            ColliderDesc::offset_rectangle(direction.length() / 2.0 + 0.05, thickness, center, angle).friction(0.9),
        );
    }
}

struct Car {
    chassis: BodyHandle,
    wheel_motors: Vec<JointHandle>,
}

fn add_car(world: &mut World, position: Vec2) -> Car {
    let chassis = world.add_body(BodyDesc::dynamic().position(position));
    world.add_collider(chassis, ColliderDesc::rectangle(1.3, 0.2));
    world.add_collider(chassis, ColliderDesc::offset_rectangle(0.6, 0.25, vec2(-0.2, 0.45), 0.0).density(0.5));

    let mut wheel_motors = Vec::new();
    for wheel_x in [-0.95, 0.95] {
        let axle = position + vec2(wheel_x, -0.5);
        // The hub slides vertically in the chassis, pushed down by a spring…
        let hub = world.add_body(BodyDesc::dynamic().position(axle));
        world.add_joint(PrismaticJointDesc::at_world_point(world, chassis, hub, axle, Vec2::Y).limits(-0.2..=0.2));
        world.add_joint(
            DistanceJointDesc::between_world_points(world, chassis, hub, axle + vec2(0.0, 0.5), axle).spring(4.0, 0.6),
        );
        // …and the wheel turns on the hub.
        let wheel = world.add_body(BodyDesc::dynamic().position(axle));
        world.add_collider(wheel, ColliderDesc::circle(WHEEL_RADIUS).friction(1.0));
        wheel_motors
            .push(world.add_joint(RevoluteJointDesc::at_world_point(world, hub, wheel, axle).motor(0.0, IDLE_TORQUE)));
    }
    Car { chassis, wheel_motors }
}

impl SceneUpdate for Car {
    fn update(&mut self, world: &mut World, _time: f32) {
        // Clockwise wheels drive the car to the right.
        let motor = match (is_key_down(KeyCode::Up), is_key_down(KeyCode::Down)) {
            (true, false) => Motor { speed: -DRIVE_SPEED, max_force: DRIVE_TORQUE },
            (false, true) => Motor { speed: DRIVE_SPEED, max_force: DRIVE_TORQUE },
            _ => Motor { speed: 0.0, max_force: IDLE_TORQUE },
        };
        for &joint in &self.wheel_motors {
            if let Some(JointKind::Revolute(revolute)) = world.joint_mut(joint).map(kinetica::Joint::kind_mut) {
                revolute.set_motor(Some(motor));
            }
        }
    }

    fn camera_target(&self, world: &World) -> Option<Vec2> {
        world.body(self.chassis).map(|chassis| chassis.position() + vec2(0.0, 1.0))
    }
}
