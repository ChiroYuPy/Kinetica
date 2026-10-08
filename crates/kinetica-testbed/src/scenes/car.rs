use kinetica::joint::Motor;
use kinetica::prelude::*;
use macroquad::input::{KeyCode, is_key_down};

use super::{Build, Category, Scene, SceneUpdate};

pub const SCENE: Scene = Scene {
    name: "Car",
    category: Category::Joints,
    description: "Left/Right to drive. Wheels: revolute motors on hubs sliding on prismatic joints held by springs.",
    camera_center: vec2(START.x, START.y + 1.0),
    view_height: 12.0,
    build: Build::Animated(build),
};

const START: Vec2 = vec2(-6.0, 1.5);
const CHASSIS_HALF_LENGTH: f32 = 1.8;
const CHASSIS_HALF_HEIGHT: f32 = 0.2;
const WHEEL_RADIUS: f32 = 0.45;
/// The hub needs a collider: a body without one has no rotational inertia and could not
/// rotate, and the prismatic joint would then stop the whole chassis from rotating too. This
/// small disc stays inside the wheel, so it never touches anything.
const HUB_RADIUS: f32 = 0.15;
/// Horizontal distance from the chassis center to each axle.
const AXLE_X: f32 = 1.35;
/// Axle height below the chassis center when the suspension is at rest.
const AXLE_DROP: f32 = 0.8;
/// How far the axle may move up (compression) and down (extension) from rest. Even fully
/// compressed, the wheel stays below the chassis: they would collide otherwise.
const SUSPENSION_TRAVEL: std::ops::RangeInclusive<f32> = -0.2..=0.12;
/// The chassis is much heavier than the wheels, like a real car: otherwise the motors' reaction
/// torque would flip it over.
const CHASSIS_DENSITY: f32 = 5.0;
const DRIVE_SPEED: f32 = 20.0;
const DRIVE_TORQUE: f32 = 40.0;
/// Small torque holding the wheels when no key is pressed, like rolling resistance.
const IDLE_TORQUE: f32 = 2.0;

fn build(world: &mut World) -> Box<dyn SceneUpdate> {
    add_terrain(world);
    Box::new(add_car(world, START))
}

/// Rolling hills made of rectangles laid end to end.
fn add_terrain(world: &mut World) {
    let ground = world.add_body(BodyDesc::fixed());
    let heights = [
        0.0, 0.0, 0.0, 0.0, 0.3, 1.0, 1.8, 2.0, 1.5, 0.5, 0.0, -0.3, 0.0, 0.8, 0.8, 0.0, 0.0, 0.6, 1.4, 2.2, 2.8, 3.0,
        2.6, 1.6, 0.6, 0.0, 0.0, 0.0,
    ];
    let start_x = -10.0;
    let segment_width = 2.0;
    let points: Vec<Vec2> =
        heights.iter().enumerate().map(|(i, &height)| vec2(start_x + segment_width * i as f32, height)).collect();
    // Walls at both ends keep the car on the track.
    for end in [points[0], points[points.len() - 1]] {
        world.add_collider(ground, ColliderDesc::offset_rectangle(0.25, 2.0, end + vec2(0.0, 1.5), 0.0));
    }
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
    world.add_collider(
        chassis,
        ColliderDesc::rectangle(CHASSIS_HALF_LENGTH, CHASSIS_HALF_HEIGHT).density(CHASSIS_DENSITY),
    );
    world.add_collider(chassis, ColliderDesc::offset_rectangle(0.8, 0.25, vec2(-0.3, 0.45), 0.0).density(1.0));

    let clearance = AXLE_DROP - SUSPENSION_TRAVEL.end() - WHEEL_RADIUS - CHASSIS_HALF_HEIGHT;
    debug_assert!(clearance > 0.0, "fully compressed, the wheels would hit the chassis");

    let mut wheel_motors = Vec::new();
    for side in [-1.0, 1.0] {
        let axle = position + vec2(side * AXLE_X, -AXLE_DROP);
        // The hub slides vertically in the chassis, pushed down by a spring…
        let hub = world.add_body(BodyDesc::dynamic().position(axle));
        world.add_collider(hub, ColliderDesc::circle(HUB_RADIUS).density(5.0));
        world.add_joint(
            PrismaticJointDesc::at_world_point(world, chassis, hub, axle, Vec2::Y).limits(SUSPENSION_TRAVEL),
        );
        world.add_joint(
            DistanceJointDesc::between_world_points(world, chassis, hub, axle + vec2(0.0, AXLE_DROP), axle)
                .spring(4.0, 0.6),
        );
        // …and the wheel turns on the hub.
        let wheel = world.add_body(BodyDesc::dynamic().position(axle));
        world.add_collider(wheel, ColliderDesc::circle(WHEEL_RADIUS).friction(1.0));
        wheel_motors
            .push(world.add_joint(RevoluteJointDesc::at_world_point(world, hub, wheel, axle).motor(0.0, IDLE_TORQUE)));
    }
    Car { chassis, wheel_motors }
}

impl Car {
    /// Drives forward (`1`), backward (`-1`), or lets the wheels idle (`0`).
    fn drive(&self, world: &mut World, direction: f32) {
        // Clockwise wheels drive the car to the right.
        let motor = if direction == 0.0 {
            Motor { speed: 0.0, max_force: IDLE_TORQUE }
        } else {
            Motor { speed: -direction * DRIVE_SPEED, max_force: DRIVE_TORQUE }
        };
        for &joint in &self.wheel_motors {
            if let Some(JointKind::Revolute(revolute)) = world.joint_mut(joint).map(kinetica::Joint::kind_mut) {
                revolute.set_motor(Some(motor));
            }
        }
    }
}

impl SceneUpdate for Car {
    fn update(&mut self, world: &mut World, _time: f32) {
        let direction = match (is_key_down(KeyCode::Right), is_key_down(KeyCode::Left)) {
            (true, false) => 1.0,
            (false, true) => -1.0,
            _ => 0.0,
        };
        self.drive(world, direction);
    }

    fn camera_target(&self, world: &World) -> Option<Vec2> {
        world.body(self.chassis).map(|chassis| chassis.center_of_mass() + vec2(0.0, 1.0))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Lets the car settle on flat ground without input, then checks how its wheels touch things.
    #[test]
    fn wheels_rest_on_the_ground_and_never_touch_the_chassis() {
        let mut world = World::new();
        add_terrain(&mut world);
        let car = add_car(&mut world, vec2(-6.0, 1.5));
        for _ in 0..300 {
            world.step(1.0 / 60.0);
        }

        let wheels: Vec<BodyHandle> =
            car.wheel_motors.iter().map(|&joint| world.joint(joint).unwrap().body_b()).collect();
        for contact in world.contacts() {
            let pair = contact.pair();
            for &wheel in &wheels {
                if pair.involves_body(wheel) {
                    assert!(!pair.involves_body(car.chassis), "a wheel rubs against the chassis");
                    let deepest = contact
                        .manifold()
                        .points()
                        .iter()
                        .map(kinetica::collision::ManifoldPoint::separation)
                        .fold(0.0, f32::min);
                    assert!(deepest > -0.02, "a wheel sinks {deepest} m into the ground");
                }
            }
        }
        assert!(
            wheels.iter().all(|&wheel| world.body_contacts(wheel).count() > 0),
            "every wheel should touch the ground"
        );
    }

    /// On the first hill (about 20° between x = 0 and x = 2), the car must tilt with the slope.
    #[test]
    fn car_tilts_on_a_slope() {
        let mut world = World::new();
        add_terrain(&mut world);
        let car = add_car(&mut world, vec2(1.0, 3.0));
        for _ in 0..120 {
            world.step(1.0 / 60.0);
        }
        let angle = world.body(car.chassis).unwrap().angle();
        assert!(angle > 0.15, "the chassis stays level on a slope: angle {angle}");
    }

    #[test]
    fn car_drives_the_whole_track_without_flipping() {
        let mut world = World::new();
        add_terrain(&mut world);
        let car = add_car(&mut world, START);
        let end_of_track = -10.0 + 2.0 * 27.0;
        let finish = end_of_track - 6.0;
        for _ in 0..12 * 60 {
            car.drive(&mut world, 1.0);
            world.step(1.0 / 60.0);
            let chassis = world.body(car.chassis).unwrap();
            assert!(chassis.angle().abs() < 1.0, "the car flipped over at {:?}", chassis.position());
            if chassis.position().x > finish {
                return;
            }
        }
        panic!("the car did not reach x = {finish}, it is at {:?}", world.body(car.chassis).unwrap().position());
    }
}
