//! Interactive testbed for the Kinetica physics engine.

mod app;
mod camera;
mod render;
mod scenes;

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
    let mut app = App::new();
    loop {
        app.update();
        app.draw();
        next_frame().await;
    }
}
