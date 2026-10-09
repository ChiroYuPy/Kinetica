//! The character controller: running, jumping and riding moving platforms.

use kinetica::prelude::*;

/// Top running speed, in m/s.
const RUN_SPEED: f32 = 7.0;
/// How fast the running speed changes, in m/s², on the ground and in the air.
const GROUND_ACCELERATION: f32 = 60.0;
const AIR_ACCELERATION: f32 = 25.0;
/// Upward speed of a jump, in m/s: about 3.7 m high with a full press.
const JUMP_SPEED: f32 = 8.5;
/// A jump is still allowed this long after running off an edge, in seconds.
const COYOTE_TIME: f32 = 0.1;
/// A jump pressed this long before landing still happens on landing, in seconds.
const JUMP_BUFFER_TIME: f32 = 0.12;
/// Gravity multipliers: falling is faster than rising, and releasing the jump early cuts it.
const FALL_GRAVITY_SCALE: f32 = 1.8;
const SHORT_JUMP_GRAVITY_SCALE: f32 = 3.0;
pub const HALF_LENGTH: f32 = 0.3;
pub const RADIUS: f32 = 0.3;

/// What the player asks for this step.
#[derive(Debug, Clone, Copy, Default)]
pub struct Input {
    /// -1 (left) to 1 (right).
    pub run: f32,
    /// The jump button went down this step.
    pub jump_pressed: bool,
    /// The jump button is held.
    pub jump_held: bool,
}

#[derive(Debug, Clone)]
pub struct Player {
    pub body: BodyHandle,
    feet: ColliderHandle,
    coyote_timer: f32,
    jump_buffer: f32,
}

impl Player {
    pub fn new(world: &mut World, position: Vec2) -> Self {
        let body = world.add_body(BodyDesc::dynamic().position(position).fixed_rotation(true));
        // No friction: the controller sets the running speed itself, and the character does not
        // stick to walls.
        world.add_collider(body, ColliderDesc::vertical_capsule(HALF_LENGTH, RADIUS).friction(0.0).density(2.0));
        let feet_center = vec2(0.0, -(HALF_LENGTH + RADIUS));
        let feet = world.add_collider(body, ColliderDesc::offset_rectangle(0.2, 0.06, feet_center, 0.0).sensor(true));
        Self { body, feet, coyote_timer: 0.0, jump_buffer: 0.0 }
    }

    /// The body the player stands on, if any. Bodies passing up through the feet, like a
    /// one-way platform during a jump, do not count.
    pub fn ground(&self, world: &World) -> Option<BodyHandle> {
        let velocity = world.body(self.body)?.linear_velocity();
        world.sensor_overlaps(self.feet).map(|pair| pair.visitor_body).find(|&ground| {
            let ground_velocity = world.body(ground).map_or(Vec2::ZERO, kinetica::Body::linear_velocity);
            velocity.y - ground_velocity.y < 0.5
        })
    }

    pub fn update(&mut self, world: &mut World, input: Input, dt: f32) {
        let ground = self.ground(world);
        // Moving platforms carry the player: speeds are relative to them.
        let ground_velocity = ground.and_then(|ground| world.body(ground)).map_or(Vec2::ZERO, |ground| {
            if ground.body_type() == BodyType::Fixed { Vec2::ZERO } else { ground.linear_velocity() }
        });

        self.coyote_timer = if ground.is_some() { COYOTE_TIME } else { self.coyote_timer - dt };
        self.jump_buffer = if input.jump_pressed { JUMP_BUFFER_TIME } else { self.jump_buffer - dt };

        let Some(body) = world.body_mut(self.body) else { return };
        let mut velocity = body.linear_velocity();
        let target = input.run.clamp(-1.0, 1.0) * RUN_SPEED + ground_velocity.x;
        let acceleration = if ground.is_some() { GROUND_ACCELERATION } else { AIR_ACCELERATION };
        velocity.x += (target - velocity.x).clamp(-acceleration * dt, acceleration * dt);

        if self.jump_buffer > 0.0 && self.coyote_timer > 0.0 {
            velocity.y = JUMP_SPEED + ground_velocity.y.max(0.0);
            self.jump_buffer = 0.0;
            self.coyote_timer = 0.0;
        }
        let gravity_scale = if velocity.y < 0.0 {
            FALL_GRAVITY_SCALE
        } else if velocity.y > 0.0 && !input.jump_held && ground.is_none() {
            SHORT_JUMP_GRAVITY_SCALE
        } else {
            1.0
        };

        // Only touch the body when something changes, so a still player can fall asleep.
        if velocity != body.linear_velocity() {
            body.set_linear_velocity(velocity);
        }
        if (gravity_scale - body.gravity_scale()).abs() > f32::EPSILON {
            body.set_gravity_scale(gravity_scale);
        }
    }

    pub fn position(&self, world: &World) -> Vec2 {
        world.body(self.body).map_or(Vec2::ZERO, kinetica::Body::position)
    }

    /// Puts the player back at `position`, at rest.
    pub fn respawn(&mut self, world: &mut World, position: Vec2) {
        if let Some(body) = world.body_mut(self.body) {
            body.set_position(position);
            body.set_linear_velocity(Vec2::ZERO);
        }
        self.coyote_timer = 0.0;
        self.jump_buffer = 0.0;
    }
}
