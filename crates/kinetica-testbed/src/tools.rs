//! What the left mouse button does to the world: drag bodies, launch new ones, or blow them away.

use kinetica::math::Rot;
use kinetica::prelude::*;
use kinetica_macroquad::{DebugRenderer, MouseGrab};
use macroquad::color::Color;
use macroquad::input::{KeyCode, MouseButton, is_mouse_button_down, is_mouse_button_pressed};

/// Launch speed for each meter the mouse was dragged, in m/s.
const LAUNCH_SPEED_PER_METER: f32 = 6.0;
/// Launched bodies are dense, so they push what they hit.
const PROJECTILE_DENSITY: f32 = 4.0;
/// Bodies closer than this to a blast are pushed away, in meters.
const BLAST_RADIUS: f32 = 4.0;
/// Speed a blast gives to a body at its center, in m/s; it fades to zero at [`BLAST_RADIUS`].
const BLAST_SPEED: f32 = 14.0;
/// How long the ring of a blast stays on screen, in seconds.
const BLAST_FLASH: f32 = 0.25;
const TOOL_COLOR: Color = Color::new(0.95, 0.95, 0.4, 1.0);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Tool {
    Grab,
    Launch,
    Blast,
}

impl Tool {
    pub const ALL: [Self; 3] = [Self::Grab, Self::Launch, Self::Blast];

    pub fn label(self) -> &'static str {
        match self {
            Self::Grab => "Grab",
            Self::Launch => "Launch",
            Self::Blast => "Blast",
        }
    }

    /// The key selecting the tool.
    pub fn key(self) -> KeyCode {
        match self {
            Self::Grab => KeyCode::G,
            Self::Launch => KeyCode::L,
            Self::Blast => KeyCode::B,
        }
    }

    /// What the left mouse button does with the tool.
    pub fn hint(self) -> &'static str {
        match self {
            Self::Grab => "Drag a body to pull it",
            Self::Launch => "Click to drop a body, drag to throw it",
            Self::Blast => "Click to blow bodies away",
        }
    }
}

/// The body thrown by [`Tool::Launch`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Projectile {
    Ball,
    Box,
    Capsule,
}

impl Projectile {
    pub const ALL: [Self; 3] = [Self::Ball, Self::Box, Self::Capsule];

    pub fn label(self) -> &'static str {
        match self {
            Self::Ball => "Ball",
            Self::Box => "Box",
            Self::Capsule => "Capsule",
        }
    }

    fn collider(self) -> ColliderDesc {
        match self {
            Self::Ball => ColliderDesc::circle(0.35),
            Self::Box => ColliderDesc::square(0.6),
            Self::Capsule => ColliderDesc::capsule(vec2(-0.3, 0.0), vec2(0.3, 0.0), 0.2),
        }
    }
}

/// The selected tool and what it is doing.
pub struct Tools {
    pub tool: Tool,
    pub projectile: Projectile,
    grab: MouseGrab,
    /// Where the body about to be launched will appear.
    launch_start: Option<Vec2>,
    /// Center of the last blast and how long its ring stays visible, in seconds.
    blast: Option<(Vec2, f32)>,
}

impl Default for Tools {
    fn default() -> Self {
        Self {
            tool: Tool::Grab,
            projectile: Projectile::Ball,
            grab: MouseGrab::default(),
            launch_start: None,
            blast: None,
        }
    }
}

impl Tools {
    /// Reacts to the left mouse button, `mouse` being in world coordinates. With
    /// `accept_new_input` false, for example while the mouse is over a panel, nothing new starts.
    pub fn update(&mut self, world: &mut World, mouse: Vec2, accept_new_input: bool, frame_time: f32) {
        let pressed = is_mouse_button_pressed(MouseButton::Left) && accept_new_input;
        self.grab.update(world, mouse, accept_new_input && self.tool == Tool::Grab);

        if pressed && self.tool == Tool::Launch {
            self.launch_start = Some(mouse);
        }
        if let Some(start) = self.launch_start
            && !is_mouse_button_down(MouseButton::Left)
        {
            self.launch_start = None;
            world.add_body(
                BodyDesc::dynamic()
                    .position(start)
                    .linear_velocity(LAUNCH_SPEED_PER_METER * (mouse - start))
                    .bullet()
                    .collider(self.projectile.collider().density(PROJECTILE_DENSITY)),
            );
        }

        if pressed && self.tool == Tool::Blast {
            blast(world, mouse);
            self.blast = Some((mouse, BLAST_FLASH));
        }
        self.blast = self.blast.map(|(center, left)| (center, left - frame_time)).filter(|&(_, left)| left > 0.0);
    }

    /// Lets go of everything, before the world is saved or replaced.
    pub fn release(&mut self, world: &mut World) {
        self.grab.release(world);
        self.launch_start = None;
    }

    /// Draws what the tool is about to do.
    pub fn draw(&self, mouse: Vec2, drawer: &mut DebugRenderer) {
        if let Some(start) = self.launch_start {
            drawer.draw_point(start, TOOL_COLOR);
            drawer.draw_segment(start, mouse, TOOL_COLOR);
        }
        if let Some((center, left)) = self.blast {
            let age = 1.0 - left / BLAST_FLASH;
            drawer.draw_circle(center, age * BLAST_RADIUS, Rot::IDENTITY, Color { a: 1.0 - age, ..TOOL_COLOR });
        }
    }
}

/// Pushes the dynamic bodies around `center` away from it, the closest ones the hardest.
fn blast(world: &mut World, center: Vec2) {
    let pushes: Vec<(BodyHandle, Vec2)> = world
        .bodies()
        .filter(|(_, body)| body.body_type() == BodyType::Dynamic)
        .filter_map(|(handle, body)| {
            let offset = body.center_of_mass() - center;
            let strength = 1.0 - offset.length() / BLAST_RADIUS;
            (strength > 0.0).then(|| (handle, body.mass() * BLAST_SPEED * strength * offset.normalize_or(Vec2::Y)))
        })
        .collect();
    for (handle, impulse) in pushes {
        world[handle].apply_linear_impulse(impulse);
    }
}
