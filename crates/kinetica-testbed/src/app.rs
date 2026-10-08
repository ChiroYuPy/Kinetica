//! Testbed state: the current scene, simulation clock, input and HUD.

use kinetica::prelude::*;
use kinetica::{DebugDrawOptions, MouseJointDesc};
use macroquad::prelude::*;

use crate::camera::WorldCamera;
use crate::render::MacroquadDraw;
use crate::scenes::{SCENES, Scene, SceneUpdate};

/// The simulation always advances by this fixed amount, whatever the frame rate.
const TIME_STEP: f32 = 1.0 / 60.0;
/// Caps the catch-up after a slow frame, so the testbed never freezes trying to keep up.
const MAX_STEPS_PER_FRAME: f32 = 4.0;
/// A grabbed body is pulled with a force of at most this many times its weight-ish mass.
const GRAB_FORCE_PER_KILOGRAM: f32 = 1000.0;

const BACKGROUND: Color = Color::new(0.09, 0.1, 0.12, 1.0);
const HUD_TEXT: Color = Color::new(0.85, 0.87, 0.9, 1.0);
const HUD_HINT: Color = Color::new(0.55, 0.58, 0.62, 1.0);
const HUD_HIGHLIGHT: Color = Color::new(0.95, 0.75, 0.35, 1.0);
const HUD_FONT_SIZE: f32 = 22.0;
const HUD_LINE_HEIGHT: f32 = 22.0;
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
    show_scene_list: bool,
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
            camera: WorldCamera::new(scene.camera_center, scene.view_height),
            paused: false,
            show_scene_list: false,
            accumulator: 0.0,
            time: 0.0,
            draw_options: DebugDrawOptions { contacts: true, ..DebugDrawOptions::default() },
            last_step_ms: 0.0,
        }
    }

    pub fn update(&mut self) {
        self.handle_keys();
        if !self.show_scene_list || !self.handle_scene_list_click() {
            self.handle_grab();
        }
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
        if self.show_scene_list {
            self.draw_scene_list();
        }
    }

    fn scene(&self) -> &'static Scene {
        &SCENES[self.scene_index]
    }

    fn load_scene(&mut self, index: usize) {
        let keep_options = self.draw_options;
        *self = Self::new(index);
        self.draw_options = keep_options;
    }

    fn handle_keys(&mut self) {
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
        if is_key_pressed(KeyCode::Tab) {
            self.show_scene_list = !self.show_scene_list;
        }
        if is_key_pressed(KeyCode::A) {
            self.draw_options.aabbs = !self.draw_options.aabbs;
        }
        if is_key_pressed(KeyCode::C) {
            self.draw_options.centers_of_mass = !self.draw_options.centers_of_mass;
        }
        if is_key_pressed(KeyCode::K) {
            self.draw_options.contacts = !self.draw_options.contacts;
        }
        if is_key_pressed(KeyCode::J) {
            self.draw_options.joints = !self.draw_options.joints;
        }
    }

    /// Left mouse button: drag a dynamic body with a mouse joint.
    fn handle_grab(&mut self) {
        let mouse = self.camera.screen_to_world(Vec2::from(mouse_position()));

        if is_mouse_button_pressed(MouseButton::Left) && self.grab.is_none() {
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
            self.camera.set_center(target);
        }
    }

    fn draw_hud(&self) {
        let scene = self.scene();
        let status = if self.paused { "  [PAUSED]" } else { "" };
        let lines = [
            (format!("{}  ({}/{}){status}", scene.name, self.scene_index + 1, SCENES.len()), HUD_TEXT),
            (scene.description.to_owned(), HUD_HINT),
            (
                format!(
                    "bodies: {}   joints: {}   contacts: {}   events: {}   step: {:.3} ms   fps: {}",
                    self.world.body_count(),
                    self.world.joint_count(),
                    self.world.contacts().count(),
                    self.world.collision_events().len(),
                    self.last_step_ms,
                    get_fps()
                ),
                HUD_TEXT,
            ),
            (
                "Space pause  S step  R reset  Left/Right scene  Tab list  A aabbs  C centers  K contacts  \
                 J joints  left drag grab  right drag pan  wheel zoom"
                    .to_owned(),
                HUD_HINT,
            ),
        ];
        for (row, (text, color)) in lines.iter().enumerate() {
            draw_text(text, 12.0, 24.0 + HUD_LINE_HEIGHT * row as f32, HUD_FONT_SIZE, *color);
        }
    }

    /// Top-left corner of the first entry of the scene list, in screen pixels.
    fn scene_list_origin() -> Vec2 {
        vec2(24.0, 130.0)
    }

    fn draw_scene_list(&self) {
        let origin = Self::scene_list_origin();
        let height = HUD_LINE_HEIGHT * SCENES.len() as f32 + 16.0;
        draw_rectangle(origin.x - 12.0, origin.y - 22.0, 280.0, height, Color::new(0.0, 0.0, 0.0, 0.75));
        for (index, scene) in SCENES.iter().enumerate() {
            let color = if index == self.scene_index { HUD_HIGHLIGHT } else { HUD_TEXT };
            let y = origin.y + HUD_LINE_HEIGHT * index as f32;
            draw_text(format!("{:>2}  {}", index + 1, scene.name), origin.x, y, HUD_FONT_SIZE, color);
        }
    }

    /// Loads the scene clicked in the list; returns `true` if the click was used.
    fn handle_scene_list_click(&mut self) -> bool {
        if !is_mouse_button_pressed(MouseButton::Left) {
            return false;
        }
        let (x, y) = mouse_position();
        let origin = Self::scene_list_origin();
        let row = ((y - origin.y + HUD_LINE_HEIGHT) / HUD_LINE_HEIGHT).floor();
        if x < origin.x - 12.0 || x > origin.x + 268.0 || row < 0.0 {
            return false;
        }
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss, reason = "row is a small positive integer")]
        let index = row as usize;
        if index >= SCENES.len() {
            return false;
        }
        self.load_scene(index);
        true
    }
}
