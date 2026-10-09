//! Interactive testbed for the Kinetica physics engine.
//!
//! Usage: `kinetica-testbed [scene number]`, scenes being numbered from 1.
//!
//! With `KINETICA_SCREENSHOTS=<directory>` set, it instead saves a picture of every scene after a
//! few seconds of simulation, then exits.

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
    if let Ok(directory) = std::env::var("KINETICA_SCREENSHOTS") {
        save_screenshots(std::path::Path::new(&directory)).await;
        return;
    }
    let scene_number = std::env::args().nth(1).and_then(|arg| arg.parse::<usize>().ok()).unwrap_or(1);
    let mut app = App::new(scene_number.saturating_sub(1));
    loop {
        app.update();
        app.draw();
        next_frame().await;
    }
}

/// Steps simulated before each screenshot: 3 s.
const SCREENSHOT_STEPS: usize = 180;

async fn save_screenshots(directory: &std::path::Path) {
    for (index, scene) in scenes::SCENES.iter().enumerate() {
        let mut app = App::new(index);
        app.simulate(SCREENSHOT_STEPS);
        // egui lays the panel out during the first frame.
        for frame in 0..2 {
            app.update();
            app.draw();
            if frame == 1 {
                let name = scene.name.to_lowercase().replace([' ', '\''], "-");
                let path = directory.join(format!("{:02}-{name}.png", index + 1));
                macroquad::texture::get_screen_data().export_png(&path.to_string_lossy());
            }
            next_frame().await;
        }
    }
}
