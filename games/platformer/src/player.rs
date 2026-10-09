//! The character controller: running, jumping, and riding moving platforms.

use kinetica::prelude::*;

/// Top running speed, in m/s.
const RUN_SPEED: f32 = 6.5;
/// How fast the running speed changes, in m/s², on the ground and in the air.
const GROUND_ACCELERATION: f32 = 60.0;
const AIR_ACCELERATION: f32 = 30.0;
/// Upward speed of a jump, in m/s: about 2.9 m high when the button is held.
const JUMP_SPEED: f32 = 7.5;
/// A jump still works this long after running off an edge, in seconds.
const COYOTE_TIME: f32 = 0.1;
/// A jump pressed this long before landing happens on landing, in seconds.
const JUMP_BUFFER: f32 = 0.12;
/// Gravity multipliers: falls are quicker than rises, and letting go of the button cuts a jump.
const FALL_GRAVITY_SCALE: f32 = 1.8;
const SHORT_JUMP_GRAVITY_SCALE: f32 = 3.0;
/// Size of the capsule.
pub const HEIGHT: f32 = 1.3;
pub const RADIUS: f32 = 0.35;

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
    /// A thin sensor under the capsule, touching whatever the player stands on.
    feet: ColliderHandle,
    /// Which way the player looks: -1 left, 1 right.
    pub facing: f32,
    coyote_timer: f32,
    jump_buffer: f32,
}

impl Player {
    pub fn new(world: &mut World, position: Vec2) -> Self {
        // No friction: the controller sets the running speed itself, and the player does not
        // stick to walls.
        let shape = ColliderDesc::vertical_capsule(HEIGHT, RADIUS).friction(0.0);
        let body = world.add_body(BodyDesc::dynamic().position(position).lock_rotation().collider(shape));
        let feet = ColliderDesc::rectangle(1.2 * RADIUS, 0.1).offset(vec2(0.0, -HEIGHT / 2.0)).sensor();
        let feet = world.add_collider(body, feet);
        Self { body, feet, facing: 1.0, coyote_timer: 0.0, jump_buffer: 0.0 }
    }

    /// The body the player stands on, if any. A body passing up through the feet, like a
    /// one-way platform during a jump, does not count.
    pub fn ground(&self, world: &World) -> Option<BodyHandle> {
        let velocity = world[self.body].linear_velocity();
        world
            .sensor_overlaps(self.feet)
            .map(|pair| pair.visitor_body)
            .find(|&ground| velocity.y - world[ground].linear_velocity().y < 0.5)
    }

    pub fn update(&mut self, world: &mut World, input: Input, dt: f32) {
        let ground = self.ground(world);
        // Moving platforms carry the player: running speeds are relative to them.
        let ground_velocity = ground.map_or(Vec2::ZERO, |ground| world[ground].linear_velocity());
        self.coyote_timer = if ground.is_some() { COYOTE_TIME } else { self.coyote_timer - dt };
        self.jump_buffer = if input.jump_pressed { JUMP_BUFFER } else { self.jump_buffer - dt };
        if input.run != 0.0 {
            self.facing = input.run.signum();
        }

        let body = &mut world[self.body];
        let mut velocity = body.linear_velocity();
        let target = input.run.clamp(-1.0, 1.0) * RUN_SPEED + ground_velocity.x;
        let acceleration = if ground.is_some() { GROUND_ACCELERATION } else { AIR_ACCELERATION };
        velocity.x += (target - velocity.x).clamp(-acceleration * dt, acceleration * dt);

        if self.jump_buffer > 0.0 && self.coyote_timer > 0.0 {
            velocity.y = JUMP_SPEED + ground_velocity.y.max(0.0);
            self.jump_buffer = 0.0;
            self.coyote_timer = 0.0;
        }
        let gravity_scale = match velocity.y {
            vy if vy < 0.0 => FALL_GRAVITY_SCALE,
            vy if vy > 0.0 && !input.jump_held && ground.is_none() => SHORT_JUMP_GRAVITY_SCALE,
            _ => 1.0,
        };
        // Setting unchanged values does nothing, so a player standing still can fall asleep.
        body.set_linear_velocity(velocity);
        body.set_gravity_scale(gravity_scale);
    }

    pub fn position(&self, world: &World) -> Vec2 {
        world[self.body].position()
    }

    /// Puts the player back at `position`, at rest.
    pub fn respawn(&mut self, world: &mut World, position: Vec2) {
        let body = &mut world[self.body];
        body.set_position(position);
        body.set_linear_velocity(Vec2::ZERO);
        self.coyote_timer = 0.0;
        self.jump_buffer = 0.0;
    }
}
