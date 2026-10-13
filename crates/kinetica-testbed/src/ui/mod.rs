//! The interface around the world view: a toolbar on top, a status bar at the bottom, an options
//! panel that can be hidden, and a scene picker opened on demand.

mod options;
mod picker;
mod theme;
mod toolbar;

use egui_macroquad::egui::{self, Align, Color32, FontId, RichText, text::LayoutJob};
use kinetica::DebugDrawOptions;

pub use self::picker::Picker;
pub use self::theme::apply as apply_theme;
use crate::tools::{Projectile, Tool};

const LABEL_FONT_SIZE: f32 = 13.0;
const LABEL_COLOR: Color32 = Color32::from_rgb(217, 222, 230);
const LABEL_BACKGROUND: Color32 = Color32::from_black_alpha(115);

/// Something the user asked for, with the mouse or a key.
#[derive(Debug, Clone, Copy)]
pub enum UiAction {
    LoadScene(usize),
    PreviousScene,
    NextScene,
    PickScene,
    TogglePause,
    Step,
    Restart,
    Save,
    Restore,
    ResetView,
    ToggleOptions,
    SelectTool(Tool),
}

/// Which parts of the interface are open.
pub struct Interface {
    pub picker: Picker,
    pub options_open: bool,
}

impl Default for Interface {
    fn default() -> Self {
        Self { picker: Picker::default(), options_open: true }
    }
}

/// What the interface shows about the testbed, and the options it edits in place.
pub struct View<'a> {
    pub scene_index: usize,
    pub paused: bool,
    pub time: f32,
    pub stats: Stats,
    /// Size of the saved world, in bytes, if there is one.
    pub snapshot_size: Option<usize>,
    pub draw_options: &'a mut DebugDrawOptions,
    pub settings: &'a mut Settings,
    pub tool: &'a mut Tool,
    pub projectile: &'a mut Projectile,
}

/// Options that survive scene changes.
#[derive(Debug, Clone, Copy)]
pub struct Settings {
    pub sleeping: bool,
    pub continuous: bool,
    pub substeps: u32,
    /// How fast simulated time flows: 1 is real time, less is slow motion.
    pub speed: f32,
    pub labels: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Self { sleeping: true, continuous: true, substeps: 4, speed: 1.0, labels: true }
    }
}

pub struct Stats {
    pub bodies: usize,
    pub awake: usize,
    pub joints: usize,
    pub contacts: usize,
    pub step_ms: f64,
    pub fps: i32,
}

/// Lays the interface out and returns the action requested this frame, if any. What is left of
/// `ctx.available_rect()` afterwards is the world view.
pub fn show(ctx: &egui::Context, interface: &mut Interface, view: &mut View) -> Option<UiAction> {
    let mut action = toolbar::show(ctx, interface, view);
    status_bar(ctx, view);
    if interface.options_open {
        action = action.or(options::show(ctx, view));
    }
    action.or(interface.picker.show(ctx, view.scene_index).map(UiAction::LoadScene))
}

fn status_bar(ctx: &egui::Context, view: &View) {
    // Numbers are padded and written with fixed-width digits, so the text around them stays still.
    let numbers = |text: String| RichText::new(text).font(FontId::monospace(11.0));
    egui::TopBottomPanel::bottom("status").exact_height(26.0).show(ctx, |ui| {
        ui.horizontal_centered(|ui| {
            let time = numbers(format!("{:>6.1} s", view.time));
            ui.label(if view.paused { time.color(theme::ACCENT) } else { time.weak() });
            let hint = format!("{}   ·   Right drag: pan   ·   Wheel: zoom", view.tool.hint());
            ui.label(RichText::new(hint).small().weak());
            ui.with_layout(egui::Layout::right_to_left(Align::Center), |ui| {
                let stats = &view.stats;
                let text = format!(
                    "{:>4} bodies  {:>4} awake  {:>4} joints  {:>5} contacts  {:>6.2} ms/step  {:>3} fps",
                    stats.bodies, stats.awake, stats.joints, stats.contacts, stats.step_ms, stats.fps
                );
                ui.label(numbers(text).weak());
            });
        });
    });
}

/// Writes the labels of a scene under the panels, each centered above its point in pixels. In
/// screen space, the text stays readable at every zoom.
pub fn world_labels(ctx: &egui::Context, labels: &[(egui::Pos2, String)]) {
    let painter = ctx.layer_painter(egui::LayerId::background()).with_clip_rect(ctx.available_rect());
    for (bottom, text) in labels {
        let mut job =
            LayoutJob::simple(text.clone(), FontId::proportional(LABEL_FONT_SIZE), LABEL_COLOR, f32::INFINITY);
        job.halign = Align::Center;
        let galley = painter.layout_job(job);
        let position = *bottom - egui::vec2(0.0, galley.size().y + 2.0);
        let background = galley.rect.translate(position.to_vec2()).expand2(egui::vec2(5.0, 2.0));
        painter.rect_filled(background, 4.0, LABEL_BACKGROUND);
        painter.galley(position, galley, LABEL_COLOR);
    }
}
