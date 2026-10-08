//! A 2D camera in world units (meters, Y up) with mouse zoom and pan.

use macroquad::prelude::*;

const ZOOM_FACTOR: f32 = 1.15;
const MIN_VIEW_HEIGHT: f32 = 0.5;
const MAX_VIEW_HEIGHT: f32 = 2000.0;
/// How quickly a followed target is caught up with: the remaining distance shrinks by a factor
/// e every `1 / FOLLOW_RATE` seconds.
const FOLLOW_RATE: f32 = 3.0;

pub struct WorldCamera {
    center: Vec2,
    /// How many meters fit vertically in the window; independent of the window size.
    view_height: f32,
    /// World point held under the cursor while panning.
    pan_anchor: Option<Vec2>,
    /// Pixels hidden by a panel on the left; the view is centered on the rest of the window.
    left_margin: f32,
}

impl WorldCamera {
    pub fn new(center: Vec2, view_height: f32, left_margin: f32) -> Self {
        Self { center, view_height, pan_anchor: None, left_margin }
    }

    /// Moves smoothly towards `target`, unless the user is panning.
    pub fn follow(&mut self, target: Vec2, dt: f32) {
        if self.pan_anchor.is_none() {
            self.center += (target - self.center) * (1.0 - (-FOLLOW_RATE * dt).exp());
        }
    }

    pub fn pixels_per_meter(&self) -> f32 {
        screen_height() / self.view_height
    }

    pub fn to_camera2d(&self) -> Camera2D {
        let pixels_per_meter = self.pixels_per_meter();
        // macroquad's 2D camera points Y down by default; a negative Y zoom flips it up.
        Camera2D {
            target: self.center,
            zoom: vec2(2.0 * pixels_per_meter / screen_width(), -2.0 * pixels_per_meter / screen_height()),
            // Shift the view right by half the panel, in normalized device coordinates.
            offset: vec2(self.left_margin / screen_width(), 0.0),
            ..Camera2D::default()
        }
    }

    pub fn screen_to_world(&self, screen_point: Vec2) -> Vec2 {
        self.to_camera2d().screen_to_world(screen_point)
    }

    /// Zooms and pans with the mouse; `accept_new_input` is false while the mouse is over a panel.
    pub fn handle_input(&mut self, accept_new_input: bool) {
        let mouse = Vec2::from(mouse_position());

        let wheel = mouse_wheel().1;
        if wheel != 0.0 && accept_new_input {
            // Zoom around the cursor: the world point under it must stay under it.
            let anchor = self.screen_to_world(mouse);
            let factor = if wheel > 0.0 { 1.0 / ZOOM_FACTOR } else { ZOOM_FACTOR };
            self.view_height = (self.view_height * factor).clamp(MIN_VIEW_HEIGHT, MAX_VIEW_HEIGHT);
            self.center += anchor - self.screen_to_world(mouse);
        }

        if is_mouse_button_pressed(MouseButton::Right) && accept_new_input {
            self.pan_anchor = Some(self.screen_to_world(mouse));
        }
        if is_mouse_button_released(MouseButton::Right) {
            self.pan_anchor = None;
        }
        if let Some(anchor) = self.pan_anchor {
            self.center += anchor - self.screen_to_world(mouse);
        }
    }
}
