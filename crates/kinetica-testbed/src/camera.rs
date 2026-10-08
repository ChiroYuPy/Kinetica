//! A 2D camera in world units (meters, Y up) with mouse zoom and pan.

use macroquad::prelude::*;

const ZOOM_FACTOR: f32 = 1.15;
const MIN_VIEW_HEIGHT: f32 = 0.5;
const MAX_VIEW_HEIGHT: f32 = 2000.0;

pub struct WorldCamera {
    center: Vec2,
    /// How many meters fit vertically in the window; independent of the window size.
    view_height: f32,
    /// World point held under the cursor while panning.
    pan_anchor: Option<Vec2>,
}

impl WorldCamera {
    pub fn new(center: Vec2, view_height: f32) -> Self {
        Self { center, view_height, pan_anchor: None }
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
            ..Camera2D::default()
        }
    }

    pub fn screen_to_world(&self, screen_point: Vec2) -> Vec2 {
        self.to_camera2d().screen_to_world(screen_point)
    }

    pub fn handle_input(&mut self) {
        let mouse = Vec2::from(mouse_position());

        let wheel = mouse_wheel().1;
        if wheel != 0.0 {
            // Zoom around the cursor: the world point under it must stay under it.
            let anchor = self.screen_to_world(mouse);
            let factor = if wheel > 0.0 { 1.0 / ZOOM_FACTOR } else { ZOOM_FACTOR };
            self.view_height = (self.view_height * factor).clamp(MIN_VIEW_HEIGHT, MAX_VIEW_HEIGHT);
            self.center += anchor - self.screen_to_world(mouse);
        }

        if is_mouse_button_pressed(MouseButton::Right) {
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
