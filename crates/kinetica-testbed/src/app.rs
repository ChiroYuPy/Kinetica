//! Testbed state: the current scene, simulation clock, input and drawing.

use egui_macroquad::egui;
use kinetica::DebugDrawOptions;
use kinetica::prelude::*;
use kinetica_macroquad::{DebugRenderer, FixedTimestep, MouseGrab, WorldCamera};
use macroquad::prelude::*;

use crate::scenes::{Frame, SCENES, SceneUpdate};
use crate::ui::{self, PanelState, SimulationSettings, Stats, UiAction};

const BACKGROUND: Color = Color::new(0.09, 0.1, 0.12, 1.0);

pub struct App {
    scene_index: usize,
    world: World,
    scene_update: Option<Box<dyn SceneUpdate>>,
    grab: MouseGrab,
    camera: WorldCamera,
    paused: bool,
    /// Whether egui uses the mouse this frame; the scene ignores it then.
    pointer_over_ui: bool,
    timestep: FixedTimestep,
    /// Simulated time since the scene started.
    time: f32,
    draw_options: DebugDrawOptions,
    settings: SimulationSettings,
    last_step_ms: f64,
}

impl App {
    /// Starts on the scene at `scene_index`, or on the first scene if the index is out of range.
    pub fn new(scene_index: usize) -> Self {
        let scene_index = if scene_index < SCENES.len() { scene_index } else { 0 };
        let scene = &SCENES[scene_index];
        let (world, scene_update) = scene.build_world();
        Self {
            scene_index,
            world,
            scene_update,
            grab: MouseGrab::default(),
            camera: WorldCamera::new(scene.camera_center, scene.view_height).with_left_margin(ui::PANEL_WIDTH),
            paused: false,
            pointer_over_ui: false,
            timestep: FixedTimestep::default(),
            time: 0.0,
            draw_options: DebugDrawOptions { contacts: true, ..DebugDrawOptions::default() },
            settings: SimulationSettings::default(),
            last_step_ms: 0.0,
        }
    }

    pub fn update(&mut self) {
        if let Some(action) = self.run_ui() {
            self.apply(action);
        }
        self.apply_settings();
        self.handle_keys();
        self.handle_grab();
        self.camera.handle_zoom_and_pan(!self.pointer_over_ui);
        if !self.paused {
            self.advance(get_frame_time());
        }
    }

    pub fn draw(&self) {
        clear_background(BACKGROUND);

        self.camera.apply();
        let mut drawer = DebugRenderer::new(&self.camera);
        drawer.draw_world(&self.world, &self.draw_options);
        if let Some(update) = &self.scene_update {
            update.draw(&self.world, &self.frame(), &mut drawer);
        }

        set_default_camera();
        egui_macroquad::draw();
    }

    fn frame(&self) -> Frame {
        Frame { time: self.time, mouse: self.camera.mouse_position() }
    }

    /// Builds the side panel for this frame and returns what the user clicked.
    fn run_ui(&mut self) -> Option<UiAction> {
        let mut action = None;
        let stats = Stats {
            bodies: self.world.body_count(),
            awake: self
                .world
                .bodies()
                .filter(|(_, body)| body.body_type() != BodyType::Fixed && !body.is_sleeping())
                .count(),
            joints: self.world.joint_count(),
            contacts: self.world.contacts().count(),
            step_ms: self.last_step_ms,
            fps: get_fps(),
        };
        let mut state = PanelState {
            scene_index: self.scene_index,
            paused: self.paused,
            time: self.time,
            stats,
            draw_options: &mut self.draw_options,
            settings: &mut self.settings,
        };
        let mut pointer_over_ui = false;
        egui_macroquad::ui(|ctx: &egui::Context| {
            action = ui::side_panel(ctx, &mut state);
            pointer_over_ui = ctx.wants_pointer_input() || ctx.is_pointer_over_area();
        });
        self.pointer_over_ui = pointer_over_ui;
        action
    }

    fn apply(&mut self, action: UiAction) {
        match action {
            UiAction::LoadScene(index) => self.load_scene(index),
            UiAction::TogglePause => self.paused = !self.paused,
            UiAction::Step => {
                self.paused = true;
                self.step();
            }
            UiAction::Reset => self.load_scene(self.scene_index),
        }
    }

    fn load_scene(&mut self, index: usize) {
        let (draw_options, settings) = (self.draw_options, self.settings);
        *self = Self::new(index);
        self.draw_options = draw_options;
        self.settings = settings;
        self.apply_settings();
    }

    fn apply_settings(&mut self) {
        if self.world.sleeping_enabled() != self.settings.sleeping {
            self.world.set_sleeping_enabled(self.settings.sleeping);
        }
        self.world.set_continuous_enabled(self.settings.continuous);
    }

    /// A few shortcuts for what is used constantly; everything else is in the panel.
    fn handle_keys(&mut self) {
        if is_key_pressed(KeyCode::Space) {
            self.apply(UiAction::TogglePause);
        }
        if is_key_pressed(KeyCode::S) {
            self.apply(UiAction::Step);
        }
        if is_key_pressed(KeyCode::R) {
            self.apply(UiAction::Reset);
        }
    }

    /// Left mouse button: drag a dynamic body with a mouse joint.
    fn handle_grab(&mut self) {
        self.grab.update(&mut self.world, self.camera.mouse_position(), !self.pointer_over_ui);
    }

    /// Runs as many fixed steps as the elapsed frame time allows.
    fn advance(&mut self, frame_time: f32) {
        for _ in 0..self.timestep.advance(frame_time) {
            self.step();
        }
    }

    fn step(&mut self) {
        let frame = self.frame();
        if let Some(update) = &mut self.scene_update {
            update.update(&mut self.world, &frame);
        }
        let start = get_time();
        let dt = self.timestep.dt();
        self.world.step(dt);
        self.last_step_ms = (get_time() - start) * 1000.0;
        self.time += dt;
        if let Some(target) = self.scene_update.as_ref().and_then(|update| update.camera_target(&self.world)) {
            self.camera.follow(target, dt);
        }
    }
}
