//! The car: a body on two sprung, motorized wheels, with a driver whose head must stay off the
//! ground.

use kinetica::prelude::*;

pub const WHEEL_RADIUS: f32 = 0.45;
/// Axles relative to the chassis center, at rest.
pub const AXLES: [Vec2; 2] = [vec2(-1.15, -0.55), vec2(1.15, -0.55)];
/// The driver's head, relative to the chassis center.
pub const HEAD: Vec2 = vec2(-0.25, 1.05);
pub const HEAD_RADIUS: f32 = 0.27;
/// The cabin on top of the chassis: its offset from the chassis center and its size.
pub const CABIN_OFFSET: Vec2 = vec2(-0.2, 0.5);
pub const CABIN_SIZE: Vec2 = vec2(1.5, 0.6);
/// Fastest wheel speed, in rad/s: about 12 m/s on the ground.
const TOP_SPEED: f32 = 27.0;
const ENGINE_TORQUE: f32 = 45.0;
/// Wheels coast with a little rolling resistance when no pedal is pressed.
const ROLLING_RESISTANCE: f32 = 1.0;
/// Angular acceleration the pedals give the car in the air, in rad/s².
const AIR_CONTROL: f32 = 5.0;

/// What the player asks of the car this step.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Controls {
    /// `1` accelerates, `-1` brakes then reverses, `0` coasts. In the air, it tilts the car.
    pub throttle: f32,
}

#[derive(Debug, Clone)]
pub struct Car {
    pub chassis: BodyHandle,
    pub wheels: [BodyHandle; 2],
    joints: [JointHandle<WheelJoint>; 2],
    /// A sensor on the chassis: the car crashes when it touches the ground.
    pub head: ColliderHandle,
}

impl Car {
    /// A car with its chassis centered on `position`.
    pub fn new(world: &mut World, position: Vec2) -> Self {
        let bodywork = ColliderDesc::convex_hull(&[
            vec2(-1.7, -0.25),
            vec2(1.6, -0.25),
            vec2(1.75, 0.0),
            vec2(1.5, 0.25),
            vec2(-1.7, 0.25),
        ])
        .expect("the bodywork is convex");
        let chassis = world.add_body(
            BodyDesc::dynamic()
                .position(position)
                .collider(bodywork.density(2.0))
                .collider(ColliderDesc::rectangle(CABIN_SIZE.x, CABIN_SIZE.y).offset(CABIN_OFFSET)),
        );
        let head = world.add_collider(chassis, ColliderDesc::circle(HEAD_RADIUS).offset(HEAD).sensor());

        let [rear, front] = AXLES.map(|offset| {
            let axle = position + offset;
            let tire = ColliderDesc::circle(WHEEL_RADIUS).friction(1.2);
            let wheel = world.add_body(BodyDesc::dynamic().position(axle).collider(tire));
            let suspension = WheelJointDesc::new(chassis, wheel, axle, Vec2::Y).spring(4.5, 0.7).limits(-0.25..=0.15);
            (wheel, world.add_joint(suspension))
        });
        Self { chassis, wheels: [rear.0, front.0], joints: [rear.1, front.1], head }
    }

    /// Applies the controls; call before every step.
    pub fn drive(&self, world: &mut World, controls: Controls) {
        // Clockwise wheels drive the car to the right.
        let motor = if controls.throttle == 0.0 {
            Motor { speed: 0.0, max_force: ROLLING_RESISTANCE }
        } else {
            Motor { speed: -controls.throttle * TOP_SPEED, max_force: ENGINE_TORQUE }
        };
        for joint in self.joints {
            world[joint].set_motor(Some(motor));
        }
        // In the air the pedals tilt the car instead: accelerating lifts the nose.
        if controls.throttle != 0.0 && !self.on_ground(world) {
            let chassis = &mut world[self.chassis];
            chassis.apply_torque(controls.throttle * AIR_CONTROL * chassis.inertia());
        }
    }

    /// Whether a wheel touches something.
    pub fn on_ground(&self, world: &World) -> bool {
        self.wheels.iter().any(|&wheel| world.body_contacts(wheel).any(|contact| !contact.manifold().is_empty()))
    }

    pub fn position(&self, world: &World) -> Vec2 {
        world[self.chassis].position()
    }

    pub fn bodies(&self) -> [BodyHandle; 3] {
        [self.chassis, self.wheels[0], self.wheels[1]]
    }

    /// Removes the car from the world.
    pub fn remove(&self, world: &mut World) {
        for body in self.bodies() {
            world.remove_body(body);
        }
    }
}
