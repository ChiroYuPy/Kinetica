//! A 2D camera in world units (meters, Y up) with mouse zoom and pan.

use macroquad::prelude::*;

const ZOOM_FACTOR: f32 = 1.15;
const MIN_VIEW_HEIGHT: f32 = 0.5;
const MAX_VIEW_HEIGHT: f32 = 2000.0;
/// How quickly a followed target is caught up with: the remaining distance shrinks by a factor
/// e every `1 / FOLLOW_RATE` seconds.
const FOLLOW_RATE: f32 = 3.0;

/// A camera showing a fixed height of the world, in meters, whatever the window size.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldCamera {
    center: Vec2,
    view_height: f32,
    /// World point held under the cursor while panning.
    pan_anchor: Option<Vec2>,
    /// Part of the window the view fills, in pixels; the whole window if `None`.
    viewport: Option<Rect>,
}

impl WorldCamera {
    /// A camera looking at `center`, showing `view_height` meters vertically.
    #[must_use]
    pub fn new(center: Vec2, view_height: f32) -> Self {
        Self { center, view_height, pan_anchor: None, viewport: None }
    }

    /// Fits the view in `viewport`, a part of the window in pixels: for example what panels
    /// leave free.
    pub fn set_viewport(&mut self, viewport: Rect) {
        self.viewport = Some(viewport);
    }

    /// The part of the window the view fills, in pixels.
    #[must_use]
    pub fn viewport(&self) -> Rect {
        self.viewport.unwrap_or(Rect::new(0.0, 0.0, screen_width(), screen_height()))
    }

    /// The world point at the center of the view.
    #[must_use]
    pub fn center(&self) -> Vec2 {
        self.center
    }

    /// Looks at `center`.
    pub fn set_center(&mut self, center: Vec2) {
        self.center = center;
    }

    /// Height of the visible world, in meters.
    #[must_use]
    pub fn view_height(&self) -> f32 {
        self.view_height
    }

    /// Shows `view_height` meters vertically.
    pub fn set_view_height(&mut self, view_height: f32) {
        self.view_height = view_height.clamp(MIN_VIEW_HEIGHT, MAX_VIEW_HEIGHT);
    }

    /// Moves smoothly towards `target`, unless the user is panning.
    pub fn follow(&mut self, target: Vec2, dt: f32) {
        if self.pan_anchor.is_none() {
            self.center += (target - self.center) * (1.0 - (-FOLLOW_RATE * dt).exp());
        }
    }

    /// Size of a meter on screen, in pixels.
    #[must_use]
    pub fn pixels_per_meter(&self) -> f32 {
        self.viewport().h / self.view_height
    }

    /// The equivalent macroquad camera.
    #[must_use]
    pub fn to_camera2d(&self) -> Camera2D {
        let pixels_per_meter = self.pixels_per_meter();
        let viewport_center = self.viewport().center();
        // macroquad's 2D camera points Y down by default; a negative Y zoom flips it up.
        Camera2D {
            target: self.center,
            zoom: vec2(2.0 * pixels_per_meter / screen_width(), -2.0 * pixels_per_meter / screen_height()),
            // Where the viewport center is, in normalized device coordinates: from -1 to 1, Y up.
            offset: vec2(
                2.0 * viewport_center.x / screen_width() - 1.0,
                1.0 - 2.0 * viewport_center.y / screen_height(),
            ),
            ..Camera2D::default()
        }
    }

    /// Makes macroquad draw in world coordinates through this camera.
    pub fn apply(&self) {
        set_camera(&self.to_camera2d());
    }

    /// Converts a point in pixels (like the mouse position) to world coordinates.
    #[must_use]
    pub fn screen_to_world(&self, screen_point: Vec2) -> Vec2 {
        self.to_camera2d().screen_to_world(screen_point)
    }

    /// Converts a world point to pixels, for example to draw text over a body.
    #[must_use]
    pub fn world_to_screen(&self, world_point: Vec2) -> Vec2 {
        self.to_camera2d().world_to_screen(world_point)
    }

    /// The mouse position, in world coordinates.
    #[must_use]
    pub fn mouse_position(&self) -> Vec2 {
        self.screen_to_world(Vec2::from(macroquad::input::mouse_position()))
    }

    /// Zooms with the wheel and pans with the right button. While `accept_new_input` is false,
    /// for example while the mouse is over a panel, no new zoom or pan starts.
    pub fn handle_zoom_and_pan(&mut self, accept_new_input: bool) {
        let mouse = Vec2::from(macroquad::input::mouse_position());

        let wheel = mouse_wheel().1;
        if wheel != 0.0 && accept_new_input {
            // Zoom around the cursor: the world point under it must stay under it.
            let anchor = self.screen_to_world(mouse);
            let factor = if wheel > 0.0 { 1.0 / ZOOM_FACTOR } else { ZOOM_FACTOR };
            self.set_view_height(self.view_height * factor);
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
