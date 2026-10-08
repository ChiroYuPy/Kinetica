//! Testbed state: the current scene, simulation clock, input and HUD.

use kinetica::{DebugDrawOptions, World};
use macroquad::prelude::*;

use crate::camera::WorldCamera;
use crate::render::MacroquadDraw;
use crate::scenes::{SCENES, Scene};

/// The simulation always advances by this fixed amount, whatever the frame rate.
const TIME_STEP: f32 = 1.0 / 60.0;
/// Caps the catch-up after a slow frame, so the testbed never freezes trying to keep up.
const MAX_STEPS_PER_FRAME: f32 = 4.0;

const BACKGROUND: Color = Color::new(0.09, 0.1, 0.12, 1.0);
const HUD_TEXT: Color = Color::new(0.85, 0.87, 0.9, 1.0);
const HUD_HINT: Color = Color::new(0.55, 0.58, 0.62, 1.0);
const NUMBER_KEYS: [KeyCode; 9] = [
    KeyCode::Key1,
    KeyCode::Key2,
    KeyCode::Key3,
    KeyCode::Key4,
    KeyCode::Key5,
    KeyCode::Key6,
    KeyCode::Key7,
    KeyCode::Key8,
    KeyCode::Key9,
];

pub struct App {
    scene_index: usize,
    world: World,
    camera: WorldCamera,
    paused: bool,
    /// Simulated time owed to the world, waiting for whole steps.
    accumulator: f32,
    draw_options: DebugDrawOptions,
    last_step_ms: f64,
}

impl App {
    pub fn new() -> Self {
        let first = &SCENES[0];
        Self {
            scene_index: 0,
            world: first.build_world(),
            camera: WorldCamera::new(first.camera_center, first.view_height),
            paused: false,
            accumulator: 0.0,
            draw_options: DebugDrawOptions::default(),
            last_step_ms: 0.0,
        }
    }

    pub fn update(&mut self) {
        self.handle_input();
        self.camera.handle_input();
        if !self.paused {
            self.advance(get_frame_time());
        }
    }

    pub fn draw(&self) {
        clear_background(BACKGROUND);

        set_camera(&self.camera.to_camera2d());
        let mut drawer = MacroquadDraw::new(self.camera.pixels_per_meter());
        self.world.debug_draw(&mut drawer, &self.draw_options);

        set_default_camera();
        self.draw_hud();
    }

    fn scene(&self) -> &'static Scene {
        &SCENES[self.scene_index]
    }

    fn load_scene(&mut self, index: usize) {
        let scene = &SCENES[index];
        self.scene_index = index;
        self.world = scene.build_world();
        self.camera = WorldCamera::new(scene.camera_center, scene.view_height);
        self.accumulator = 0.0;
    }

    fn handle_input(&mut self) {
        if is_key_pressed(KeyCode::Space) {
            self.paused = !self.paused;
        }
        if is_key_pressed(KeyCode::S) {
            self.paused = true;
            self.step();
        }
        if is_key_pressed(KeyCode::R) {
            self.load_scene(self.scene_index);
        }
        if is_key_pressed(KeyCode::Right) {
            self.load_scene((self.scene_index + 1) % SCENES.len());
        }
        if is_key_pressed(KeyCode::Left) {
            self.load_scene((self.scene_index + SCENES.len() - 1) % SCENES.len());
        }
        if let Some(index) = NUMBER_KEYS.iter().position(|&key| is_key_pressed(key))
            && index < SCENES.len()
        {
            self.load_scene(index);
        }
        if is_key_pressed(KeyCode::A) {
            self.draw_options.aabbs = !self.draw_options.aabbs;
        }
        if is_key_pressed(KeyCode::C) {
            self.draw_options.centers_of_mass = !self.draw_options.centers_of_mass;
        }
    }

    /// Runs as many fixed steps as the elapsed frame time allows.
    fn advance(&mut self, frame_time: f32) {
        self.accumulator = (self.accumulator + frame_time).min(MAX_STEPS_PER_FRAME * TIME_STEP);
        while self.accumulator >= TIME_STEP {
            self.step();
            self.accumulator -= TIME_STEP;
        }
    }

    fn step(&mut self) {
        let start = get_time();
        self.world.step(TIME_STEP);
        self.last_step_ms = (get_time() - start) * 1000.0;
    }

    fn draw_hud(&self) {
        let scene = self.scene();
        let status = if self.paused { "  [PAUSED]" } else { "" };
        let lines = [
            (format!("{}  ({}/{}){status}", scene.name, self.scene_index + 1, SCENES.len()), HUD_TEXT),
            (scene.description.to_owned(), HUD_HINT),
            (
                format!(
                    "bodies: {}   colliders: {}   step: {:.3} ms   fps: {}",
                    self.world.body_count(),
                    self.world.collider_count(),
                    self.last_step_ms,
                    get_fps()
                ),
                HUD_TEXT,
            ),
            (
                "Space pause   S step   R reset   Left/Right or 1-9 scene   A aabbs   C centers   \
                 wheel zoom   right drag pan"
                    .to_owned(),
                HUD_HINT,
            ),
        ];
        for (row, (text, color)) in lines.iter().enumerate() {
            draw_text(text, 12.0, 24.0 + 22.0 * row as f32, 22.0, *color);
        }
    }
}
