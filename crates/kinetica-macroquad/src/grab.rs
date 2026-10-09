//! Dragging bodies with the mouse.

use kinetica::prelude::*;
use kinetica::{Joint, MouseJointDesc};
use macroquad::input::{MouseButton, is_mouse_button_down, is_mouse_button_pressed};

/// A grabbed body is pulled with a force of at most this many newtons per kilogram, about a
/// hundred times its weight.
const FORCE_PER_KILOGRAM: f32 = 1000.0;

/// Drags dynamic bodies with the left mouse button, through a mouse joint.
#[derive(Debug, Clone, Default)]
pub struct MouseGrab {
    /// Fixed body without colliders, the reference of the mouse joint.
    anchor: Option<BodyHandle>,
    joint: Option<JointHandle>,
}

impl MouseGrab {
    /// Grabs the dynamic body under `mouse` (in world coordinates) when the left button is
    /// pressed, drags it while the button is held, and lets go when it is released. With
    /// `accept_new_grab` false, for example while the mouse is over a panel, nothing new is
    /// grabbed.
    pub fn update(&mut self, world: &mut World, mouse: Vec2, accept_new_grab: bool) {
        if is_mouse_button_pressed(MouseButton::Left) && accept_new_grab && self.joint.is_none() {
            self.grab(world, mouse);
        }
        let Some(joint) = self.joint else { return };
        if is_mouse_button_down(MouseButton::Left) {
            if let Some(JointKind::Mouse(mouse_joint)) = world.joint_mut(joint).map(Joint::kind_mut) {
                mouse_joint.set_target(mouse);
            } else {
                // The body was removed meanwhile.
                self.joint = None;
            }
        } else {
            self.release(world);
        }
    }

    /// The body being dragged, if any.
    #[must_use]
    pub fn grabbed_body(&self, world: &World) -> Option<BodyHandle> {
        world.joint(self.joint?).map(Joint::body_b)
    }

    /// Lets go of the dragged body.
    pub fn release(&mut self, world: &mut World) {
        if let Some(joint) = self.joint.take() {
            world.remove_joint(joint);
        }
    }

    fn grab(&mut self, world: &mut World, point: Vec2) {
        let body = world
            .colliders_at_point(point, &QueryFilter::new().exclude_fixed().exclude_kinematic())
            .into_iter()
            .find_map(|collider| world.collider(collider).map(kinetica::Collider::body));
        let Some(body) = body else { return };

        // The anchor may belong to a previous world.
        let anchor = match self.anchor {
            Some(anchor) if world.body(anchor).is_some_and(|anchor| anchor.body_type() == BodyType::Fixed) => anchor,
            _ => world.add_body(BodyDesc::fixed()),
        };
        self.anchor = Some(anchor);
        let mass = world.body(body).map_or(1.0, kinetica::Body::mass);
        let desc = MouseJointDesc::at_world_point(world, anchor, body, point).max_force(FORCE_PER_KILOGRAM * mass);
        self.joint = Some(world.add_joint(desc));
    }
}
