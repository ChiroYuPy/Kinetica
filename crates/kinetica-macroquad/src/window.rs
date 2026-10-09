//! The game window: its configuration, and text centered on it.

use macroquad::color::Color;
use macroquad::text::{draw_text, measure_text};
use macroquad::window::{Conf, screen_width};

/// A 1280×720 window with sharp text on high-density screens and antialiased shapes.
///
/// `#[macroquad::main]` takes a function without arguments, so wrap it:
///
/// ```no_run
/// fn window_conf() -> macroquad::window::Conf {
///     kinetica_macroquad::window_conf("My game")
/// }
///
/// #[macroquad::main(window_conf)]
/// async fn main() {}
/// ```
#[must_use]
pub fn window_conf(title: &str) -> Conf {
    Conf {
        window_title: title.to_owned(),
        window_width: 1280,
        window_height: 720,
        high_dpi: true,
        sample_count: 4,
        ..Conf::default()
    }
}

/// Writes `text` centered horizontally, with its baseline `y` pixels from the top of the window.
pub fn draw_centered_text(text: &str, y: f32, font_size: u16, color: Color) {
    let width = measure_text(text, None, font_size, 1.0).width;
    draw_text(text, (screen_width() - width) / 2.0, y, f32::from(font_size), color);
}
