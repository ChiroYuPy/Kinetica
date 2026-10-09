//! The car: a chassis on two sprung wheels driven by motors, with a driver whose head must not
//! touch the ground.

use kinetica::joint::Motor;
use kinetica::prelude::*;

const CHASSIS_HALF_LENGTH: f32 = 1.8;
const CHASSIS_HALF_HEIGHT: f32 = 0.2;
pub const WHEEL_RADIUS: f32 = 0.5;
/// The hub needs a collider: a body without one has no rotational inertia and could not
/// rotate, and the prismatic joint would then stop the whole chassis from rotating too. This
/// small disc stays inside the wheel, so it never touches anything.
const HUB_RADIUS: f32 = 0.15;
/// Horizontal distance from the chassis center to each axle.
const AXLE_X: f32 = 1.35;
/// Axle height below the chassis center when the suspension is at rest.
const AXLE_DROP: f32 = 0.85;
/// How far the axle may move up (compression) and down (extension) from rest. Even fully
/// compressed, the wheel stays below the chassis: they would collide otherwise.
const SUSPENSION_TRAVEL: std::ops::RangeInclusive<f32> = -0.2..=0.12;
/// The chassis is much heavier than the wheels, like a real car: otherwise the motors' reaction
/// torque would flip it over.
const CHASSIS_DENSITY: f32 = 5.0;
const DRIVE_SPEED: f32 = 22.0;
const DRIVE_TORQUE: f32 = 60.0;
/// Small torque holding the wheels when no pedal is pressed, like rolling resistance.
const IDLE_TORQUE: f32 = 2.0;
/// Angular acceleration given to the chassis by the arrows while in the air, in rad/s².
const AIR_CONTROL: f32 = 4.0;
/// The driver's head, relative to the chassis center.
pub const HEAD_CENTER: Vec2 = vec2(-0.3, 1.25);
pub const HEAD_RADIUS: f32 = 0.3;

/// What the player asks of the car this step.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Controls {
    /// `1` accelerates, `-1` brakes and reverses, `0` lets the car roll.
    pub throttle: f32,
}

#[derive(Debug, Clone)]
pub struct Car {
    pub chassis: BodyHandle,
    pub wheels: [BodyHandle; 2],
    /// A sensor: the car crashes when it touches the ground.
    pub head: ColliderHandle,
    motors: [JointHandle; 2],
}

impl Car {
    pub fn new(world: &mut World, position: Vec2) -> Self {
        let chassis = world.add_body(BodyDesc::dynamic().position(position));
        world.add_collider(
            chassis,
            ColliderDesc::rectangle(CHASSIS_HALF_LENGTH, CHASSIS_HALF_HEIGHT).density(CHASSIS_DENSITY),
        );
        // The cabin, and the driver's head sticking out of it.
        world.add_collider(chassis, ColliderDesc::offset_rectangle(0.7, 0.35, vec2(-0.3, 0.5), 0.0).density(1.0));
        let head = kinetica::shape::Circle::new(HEAD_RADIUS).with_center(HEAD_CENTER);
        let head = world.add_collider(chassis, ColliderDesc::new(head).sensor(true));

        let clearance = AXLE_DROP - SUSPENSION_TRAVEL.end() - WHEEL_RADIUS - CHASSIS_HALF_HEIGHT;
        debug_assert!(clearance > 0.0, "fully compressed, the wheels would hit the chassis");

        let mut wheels = Vec::new();
        let mut motors = Vec::new();
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
            world.add_collider(wheel, ColliderDesc::circle(WHEEL_RADIUS).friction(1.2));
            let motor = RevoluteJointDesc::at_world_point(world, hub, wheel, axle).motor(0.0, IDLE_TORQUE);
            motors.push(world.add_joint(motor));
            wheels.push(wheel);
        }
        Self { chassis, wheels: [wheels[0], wheels[1]], head, motors: [motors[0], motors[1]] }
    }

    /// Applies the controls; call before every step.
    pub fn drive(&self, world: &mut World, controls: Controls) {
        // Clockwise wheels drive the car to the right.
        let motor = if controls.throttle == 0.0 {
            Motor { speed: 0.0, max_force: IDLE_TORQUE }
        } else {
            Motor { speed: -controls.throttle * DRIVE_SPEED, max_force: DRIVE_TORQUE }
        };
        for &joint in &self.motors {
            if let Some(JointKind::Revolute(revolute)) = world.joint_mut(joint).map(kinetica::Joint::kind_mut) {
                revolute.set_motor(Some(motor));
            }
        }
        // In the air, the pedals tilt the car instead: accelerating lifts the nose.
        if controls.throttle != 0.0
            && !self.on_ground(world)
            && let Some(chassis) = world.body_mut(self.chassis)
        {
            let torque = controls.throttle * AIR_CONTROL * chassis.inertia();
            chassis.apply_torque(torque);
        }
    }

    /// Whether a wheel touches something.
    pub fn on_ground(&self, world: &World) -> bool {
        self.wheels.iter().any(|&wheel| {
            world
                .body_contacts(wheel)
                .any(|contact| contact.manifold().points().iter().any(|point| point.separation() < 0.02))
        })
    }

    pub fn position(&self, world: &World) -> Vec2 {
        world.body(self.chassis).map_or(Vec2::ZERO, kinetica::Body::position)
    }

    pub fn speed(&self, world: &World) -> f32 {
        world.body(self.chassis).map_or(0.0, |chassis| chassis.linear_velocity().length())
    }

    /// Every body of the car, to tell its colliders apart from the rest.
    pub fn bodies(&self, world: &World) -> Vec<BodyHandle> {
        let mut bodies = vec![self.chassis];
        bodies.extend(self.wheels);
        bodies.extend(self.motors.iter().filter_map(|&joint| world.joint(joint).map(kinetica::Joint::body_a)));
        bodies
    }
}
