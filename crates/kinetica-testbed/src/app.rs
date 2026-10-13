//! Testbed state: the current scene, simulation clock, input and drawing.

use std::collections::HashSet;

use egui_macroquad::egui;
use kinetica::DebugDrawOptions;
use kinetica::prelude::*;
use kinetica_macroquad::{DebugRenderer, FixedTimestep, WorldCamera};
use macroquad::prelude::*;

use crate::scenes::{Frame, Labels, SCENES, SceneUpdate};
use crate::tools::{Tool, Tools};
use crate::ui::{self, Interface, Settings, Stats, UiAction, View};

const BACKGROUND: Color = Color::new(0.09, 0.1, 0.12, 1.0);

/// The keyboard shortcuts; scenes use the arrows, the digits and a few other letters.
const SHORTCUTS: [(KeyCode, UiAction); 14] = [
    (KeyCode::Tab, UiAction::PickScene),
    (KeyCode::PageUp, UiAction::PreviousScene),
    (KeyCode::PageDown, UiAction::NextScene),
    (KeyCode::Space, UiAction::TogglePause),
    (KeyCode::S, UiAction::Step),
    (KeyCode::R, UiAction::Restart),
    (KeyCode::F5, UiAction::Save),
    (KeyCode::F9, UiAction::Restore),
    (KeyCode::Home, UiAction::ResetView),
    (KeyCode::O, UiAction::ToggleOptions),
    (KeyCode::G, UiAction::SelectTool(Tool::Grab)),
    (KeyCode::L, UiAction::SelectTool(Tool::Launch)),
    (KeyCode::B, UiAction::SelectTool(Tool::Blast)),
    (KeyCode::Escape, UiAction::SelectTool(Tool::Grab)),
];

/// A copy of the simulation, serialized to JSON.
struct Snapshot {
    json: String,
    time: f32,
}

pub struct App {
    scene_index: usize,
    world: World,
    scene_update: Option<Box<dyn SceneUpdate>>,
    /// Labels written when the scene was built.
    labels: Labels,
    /// Keys pressed since the last step.
    keys_pressed: HashSet<KeyCode>,
    snapshot: Option<Snapshot>,
    /// Simulated time since the scene started.
    time: f32,
    timestep: FixedTimestep,
    last_step_ms: f64,
    paused: bool,
    camera: WorldCamera,
    tools: Tools,
    draw_options: DebugDrawOptions,
    settings: Settings,
    interface: Interface,
    /// Whether egui uses the mouse this frame; the world ignores it then.
    pointer_over_ui: bool,
    /// Whether egui uses the keyboard this frame, for the search field; the world ignores it then.
    typing_in_ui: bool,
}

impl App {
    /// Starts on the scene at `scene_index`, or on the first scene if the index is out of range.
    pub fn new(scene_index: usize) -> Self {
        let mut app = Self {
            scene_index: 0,
            world: World::new(),
            scene_update: None,
            labels: Labels::default(),
            keys_pressed: HashSet::new(),
            snapshot: None,
            time: 0.0,
            timestep: FixedTimestep::default(),
            last_step_ms: 0.0,
            paused: false,
            camera: WorldCamera::new(Vec2::ZERO, 1.0),
            tools: Tools::default(),
            draw_options: DebugDrawOptions::default(),
            settings: Settings::default(),
            interface: Interface::default(),
            pointer_over_ui: false,
            typing_in_ui: false,
        };
        app.load_scene(if scene_index < SCENES.len() { scene_index } else { 0 });
        app
    }

    pub fn update(&mut self) {
        if !self.typing_in_ui {
            for (key, action) in SHORTCUTS {
                if is_key_pressed(key) {
                    self.apply(action);
                }
            }
            self.keys_pressed.extend(get_keys_pressed());
        }
        let frame_time = get_frame_time();
        self.tools.update(&mut self.world, self.camera.mouse_position(), !self.pointer_over_ui, frame_time);
        self.camera.handle_zoom_and_pan(!self.pointer_over_ui);
        if !self.paused {
            self.advance(frame_time * self.settings.speed);
        }
        // Last, so that the labels are written where the bodies now are.
        if let Some(action) = self.run_ui() {
            self.apply(action);
        }
        self.apply_settings();
    }

    pub fn draw(&self) {
        clear_background(BACKGROUND);

        self.camera.apply();
        let mut drawer = DebugRenderer::new(&self.camera);
        drawer.draw_world(&self.world, &self.draw_options);
        if let Some(update) = &self.scene_update {
            update.draw(&self.world, &self.frame(), &mut drawer);
        }
        self.tools.draw(self.camera.mouse_position(), &mut drawer);

        set_default_camera();
        egui_macroquad::draw();
    }

    fn frame(&self) -> Frame {
        let (keys_down, keys_pressed) =
            if self.typing_in_ui { Default::default() } else { (get_keys_down(), self.keys_pressed.clone()) };
        Frame { time: self.time, mouse: self.camera.mouse_position(), keys_down, keys_pressed }
    }

    /// Builds the interface for this frame, fits the camera in the room it leaves, and returns
    /// what the user asked for.
    fn run_ui(&mut self) -> Option<UiAction> {
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
        let labels = if self.settings.labels { self.labels_on_screen() } else { Vec::new() };
        let mut view = View {
            scene_index: self.scene_index,
            paused: self.paused,
            time: self.time,
            stats,
            snapshot_size: self.snapshot.as_ref().map(|snapshot| snapshot.json.len()),
            draw_options: &mut self.draw_options,
            settings: &mut self.settings,
            tool: &mut self.tools.tool,
            projectile: &mut self.tools.projectile,
        };
        let mut action = None;
        egui_macroquad::ui(|ctx: &egui::Context| {
            action = ui::show(ctx, &mut self.interface, &mut view);
            ui::world_labels(ctx, &labels);
            let free = ctx.available_rect();
            self.camera.set_viewport(Rect::new(free.min.x, free.min.y, free.width(), free.height()));
            self.pointer_over_ui = ctx.wants_pointer_input() || ctx.is_pointer_over_area();
            self.typing_in_ui = ctx.wants_keyboard_input();
        });
        action
    }

    /// The labels of the scene, with the point in pixels each one sits on.
    fn labels_on_screen(&self) -> Vec<(egui::Pos2, String)> {
        let mut changing = Labels::default();
        if let Some(update) = &self.scene_update {
            update.labels(&self.world, &self.frame(), &mut changing);
        }
        self.labels
            .iter()
            .chain(changing.iter())
            .filter_map(|label| {
                let position = self.camera.world_to_screen(label.position(&self.world)?);
                Some((egui::pos2(position.x, position.y), label.text.clone()))
            })
            .collect()
    }

    fn apply(&mut self, action: UiAction) {
        match action {
            UiAction::LoadScene(index) => self.load_scene(index),
            UiAction::PreviousScene => self.load_scene((self.scene_index + SCENES.len() - 1) % SCENES.len()),
            UiAction::NextScene => self.load_scene((self.scene_index + 1) % SCENES.len()),
            UiAction::PickScene => self.interface.picker.open(),
            UiAction::TogglePause => self.paused = !self.paused,
            UiAction::Step => {
                self.paused = true;
                self.step();
            }
            UiAction::Restart => self.load_scene(self.scene_index),
            UiAction::Save => self.save(),
            UiAction::Restore => self.restore(),
            UiAction::ResetView => self.reset_view(),
            UiAction::ToggleOptions => self.interface.options_open = !self.interface.options_open,
            UiAction::SelectTool(tool) => self.tools.tool = tool,
        }
    }

    /// Serializes the whole world: bodies, colliders, joints, contacts and solver state.
    fn save(&mut self) {
        // A dragged body would stay pinned to the mouse position of the save.
        self.tools.release(&mut self.world);
        match serde_json::to_string(&self.world) {
            Ok(json) => self.snapshot = Some(Snapshot { json, time: self.time }),
            Err(error) => eprintln!("could not save the world: {error}"),
        }
    }

    /// Replaces the world with the saved one. Handles stay valid, so the scene keeps working.
    fn restore(&mut self) {
        let Some(snapshot) = &self.snapshot else { return };
        match serde_json::from_str::<World>(&snapshot.json) {
            Ok(world) => {
                self.tools.release(&mut self.world);
                self.world = world;
                self.time = snapshot.time;
            }
            Err(error) => eprintln!("could not restore the world: {error}"),
        }
    }

    /// Starts the scene at `index` from scratch; the options, the tool and the pause are kept.
    fn load_scene(&mut self, index: usize) {
        self.tools.release(&mut self.world);
        (self.world, self.labels, self.scene_update) = SCENES[index].build_world();
        self.scene_index = index;
        self.keys_pressed.clear();
        self.snapshot = None;
        self.time = 0.0;
        self.timestep.reset();
        self.reset_view();
        self.apply_settings();
    }

    /// Frames the scene as it was designed to be seen.
    fn reset_view(&mut self) {
        let scene = &SCENES[self.scene_index];
        self.camera.set_center(scene.camera_center);
        self.camera.set_view_height(scene.view_height);
    }

    fn apply_settings(&mut self) {
        if self.world.sleeping_enabled() != self.settings.sleeping {
            self.world.set_sleeping_enabled(self.settings.sleeping);
        }
        self.world.set_continuous_enabled(self.settings.continuous);
        self.world.set_substep_count(self.settings.substeps);
    }

    /// Runs `steps` simulation steps, whatever the frame rate.
    pub fn simulate(&mut self, steps: usize) {
        for _ in 0..steps {
            self.step();
        }
    }

    /// Runs as many fixed steps as the elapsed frame time allows.
    fn advance(&mut self, frame_time: f32) {
        for _ in 0..self.timestep.advance(frame_time) {
            self.step();
        }
    }

    fn step(&mut self) {
        let frame = self.frame();
        self.keys_pressed.clear();
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
