//! Testbed state: the current scene, simulation clock, input and drawing.

use egui_macroquad::egui;
use kinetica::prelude::*;
use kinetica::{DebugDrawOptions, MouseJointDesc};
use macroquad::prelude::*;

use crate::camera::WorldCamera;
use crate::render::MacroquadDraw;
use crate::scenes::{SCENES, SceneUpdate};
use crate::ui::{self, PanelState, Stats, UiAction};

/// The simulation always advances by this fixed amount, whatever the frame rate.
const TIME_STEP: f32 = 1.0 / 60.0;
/// Caps the catch-up after a slow frame, so the testbed never freezes trying to keep up.
const MAX_STEPS_PER_FRAME: f32 = 4.0;
/// A grabbed body is pulled with a force of at most this many times its weight-ish mass.
const GRAB_FORCE_PER_KILOGRAM: f32 = 1000.0;

const BACKGROUND: Color = Color::new(0.09, 0.1, 0.12, 1.0);

/// A body held with the mouse.
struct Grab {
    joint: JointHandle,
}

pub struct App {
    scene_index: usize,
    world: World,
    scene_update: Option<Box<dyn SceneUpdate>>,
    /// Fixed body without colliders, used as the reference of mouse joints.
    mouse_anchor: BodyHandle,
    grab: Option<Grab>,
    camera: WorldCamera,
    paused: bool,
    /// Whether egui uses the mouse this frame; the scene ignores it then.
    pointer_over_ui: bool,
    /// Simulated time owed to the world, waiting for whole steps.
    accumulator: f32,
    /// Simulated time since the scene started.
    time: f32,
    draw_options: DebugDrawOptions,
    last_step_ms: f64,
}

impl App {
    /// Starts on the scene at `scene_index`, or on the first scene if the index is out of range.
    pub fn new(scene_index: usize) -> Self {
        let scene_index = if scene_index < SCENES.len() { scene_index } else { 0 };
        let scene = &SCENES[scene_index];
        let (mut world, scene_update) = scene.build_world();
        let mouse_anchor = world.add_body(BodyDesc::fixed());
        Self {
            scene_index,
            world,
            scene_update,
            mouse_anchor,
            grab: None,
            camera: WorldCamera::new(scene.camera_center, scene.view_height, ui::PANEL_WIDTH),
            paused: false,
            pointer_over_ui: false,
            accumulator: 0.0,
            time: 0.0,
            draw_options: DebugDrawOptions { contacts: true, ..DebugDrawOptions::default() },
            last_step_ms: 0.0,
        }
    }

    pub fn update(&mut self) {
        if let Some(action) = self.run_ui() {
            self.apply(action);
        }
        self.handle_keys();
        self.handle_grab();
        self.camera.handle_input(!self.pointer_over_ui);
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
        egui_macroquad::draw();
    }

    /// Builds the side panel for this frame and returns what the user clicked.
    fn run_ui(&mut self) -> Option<UiAction> {
        let mut action = None;
        let stats = Stats {
            bodies: self.world.body_count(),
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
        let keep_options = self.draw_options;
        *self = Self::new(index);
        self.draw_options = keep_options;
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
        let mouse = self.camera.screen_to_world(Vec2::from(mouse_position()));

        if is_mouse_button_pressed(MouseButton::Left) && self.grab.is_none() && !self.pointer_over_ui {
            let grabbed = self.world.colliders_at_point(mouse).find_map(|collider| {
                let body = self.world.collider(collider)?.body();
                (self.world.body(body)?.body_type() == BodyType::Dynamic).then_some(body)
            });
            if let Some(body) = grabbed {
                let mass = self.world.body(body).map_or(1.0, kinetica::Body::mass);
                let desc = MouseJointDesc::at_world_point(&self.world, self.mouse_anchor, body, mouse)
                    .max_force(GRAB_FORCE_PER_KILOGRAM * mass);
                self.grab = Some(Grab { joint: self.world.add_joint(desc) });
            }
        }

        if let Some(grab) = &self.grab {
            if is_mouse_button_down(MouseButton::Left) {
                if let Some(JointKind::Mouse(joint)) = self.world.joint_mut(grab.joint).map(kinetica::Joint::kind_mut) {
                    joint.set_target(mouse);
                }
            } else {
                self.world.remove_joint(grab.joint);
                self.grab = None;
            }
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
        if let Some(update) = &mut self.scene_update {
            update.update(&mut self.world, self.time);
        }
        let start = get_time();
        self.world.step(TIME_STEP);
        self.last_step_ms = (get_time() - start) * 1000.0;
        self.time += TIME_STEP;
        if let Some(target) = self.scene_update.as_ref().and_then(|update| update.camera_target(&self.world)) {
            self.camera.follow(target, TIME_STEP);
        }
    }
}
