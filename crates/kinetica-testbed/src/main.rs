//! Interactive testbed for the Kinetica physics engine.
//!
//! Usage: `kinetica-testbed [scene number]`, scenes being numbered from 1.

mod app;
mod scenes;
mod ui;

use macroquad::window::{Conf, next_frame};

use crate::app::App;

fn window_conf() -> Conf {
    Conf {
        window_title: "Kinetica Testbed".to_owned(),
        window_width: 1280,
        window_height: 720,
        high_dpi: true,
        sample_count: 4,
        ..Conf::default()
    }
}

#[macroquad::main(window_conf)]
async fn main() {
    let scene_number = std::env::args().nth(1).and_then(|arg| arg.parse::<usize>().ok()).unwrap_or(1);
    let mut app = App::new(scene_number.saturating_sub(1));
    loop {
        app.update();
        app.draw();
        next_frame().await;
    }
}
