//! The side panel: scene selection, simulation controls, display options and statistics.

use egui_macroquad::egui;
use kinetica::DebugDrawOptions;

use crate::scenes::{Category, SCENES};

/// Width of the side panel, in logical pixels.
pub const PANEL_WIDTH: f32 = 250.0;

/// Something the user asked for in the panel.
#[derive(Debug, Clone, Copy)]
pub enum UiAction {
    LoadScene(usize),
    TogglePause,
    Step,
    Reset,
    Save,
    Restore,
}

/// What the panel shows, and the display options it edits in place.
pub struct PanelState<'a> {
    pub scene_index: usize,
    pub paused: bool,
    pub time: f32,
    pub stats: Stats,
    pub draw_options: &'a mut DebugDrawOptions,
    pub settings: &'a mut SimulationSettings,
    /// Size of the saved world, in bytes, if there is one.
    pub snapshot_size: Option<usize>,
}

/// World options that survive scene changes.
#[derive(Debug, Clone, Copy)]
pub struct SimulationSettings {
    pub sleeping: bool,
    pub continuous: bool,
    pub substeps: u32,
    pub labels: bool,
}

impl Default for SimulationSettings {
    fn default() -> Self {
        Self { sleeping: true, continuous: true, substeps: 4, labels: true }
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

/// Draws the panel and returns the action requested this frame, if any.
pub fn side_panel(ctx: &egui::Context, state: &mut PanelState) -> Option<UiAction> {
    let mut action = None;
    egui::SidePanel::left("kinetica").exact_width(PANEL_WIDTH).resizable(false).show(ctx, |ui| {
        ui.heading("Kinetica");
        ui.separator();

        ui.horizontal(|ui| {
            let pause_label = if state.paused { "Play" } else { "Pause" };
            if ui.button(pause_label).on_hover_text("Space").clicked() {
                action = Some(UiAction::TogglePause);
            }
            if ui.button("Step").on_hover_text("S").clicked() {
                action = Some(UiAction::Step);
            }
            if ui.button("Reset").on_hover_text("R").clicked() {
                action = Some(UiAction::Reset);
            }
        });
        ui.horizontal(|ui| {
            if ui.button("Save").on_hover_text("F5: serialize the world to JSON").clicked() {
                action = Some(UiAction::Save);
            }
            let restore = ui.add_enabled(state.snapshot_size.is_some(), egui::Button::new("Restore"));
            if restore.on_hover_text("F9: load the saved world back").clicked() {
                action = Some(UiAction::Restore);
            }
            if let Some(size) = state.snapshot_size {
                ui.label(egui::RichText::new(format!("{:.0} kB", size as f32 / 1000.0)).weak());
            }
        });
        ui.label(format!("Time: {:.1} s", state.time));
        ui.separator();

        egui::ScrollArea::vertical().max_height(ui.available_height() * 0.55).auto_shrink([false, false]).show(
            ui,
            |ui| {
                for category in Category::ALL {
                    egui::CollapsingHeader::new(category.label()).default_open(true).show(ui, |ui| {
                        for (index, scene) in SCENES.iter().enumerate().filter(|(_, scene)| scene.category == category)
                        {
                            if ui.selectable_label(index == state.scene_index, scene.name).clicked() {
                                action = Some(UiAction::LoadScene(index));
                            }
                        }
                    });
                }
            },
        );
        ui.separator();

        ui.label(egui::RichText::new(SCENES[state.scene_index].description).weak());
        ui.separator();

        ui.label("Simulate");
        let settings = &mut *state.settings;
        ui.checkbox(&mut settings.sleeping, "Sleeping").on_hover_text("Resting bodies stop being simulated");
        ui.checkbox(&mut settings.continuous, "Continuous collision")
            .on_hover_text("Fast bodies do not pass through thin ones");
        ui.add(egui::Slider::new(&mut settings.substeps, 1..=16).text("Substeps"))
            .on_hover_text("More substeps make contacts and joints stiffer: tall columns stand, chains stretch less");
        ui.separator();

        ui.label("Show");
        ui.checkbox(&mut state.settings.labels, "Labels");
        let options = &mut *state.draw_options;
        ui.checkbox(&mut options.shapes, "Shapes");
        ui.checkbox(&mut options.joints, "Joints");
        ui.checkbox(&mut options.contacts, "Contacts");
        ui.checkbox(&mut options.aabbs, "Bounding boxes");
        ui.checkbox(&mut options.centers_of_mass, "Centers of mass");
        ui.separator();

        let stats = &state.stats;
        ui.label(format!("Bodies: {} ({} awake)   Joints: {}", stats.bodies, stats.awake, stats.joints));
        ui.label(format!("Contacts: {}", stats.contacts));
        ui.label(format!("Step: {:.3} ms   {} fps", stats.step_ms, stats.fps));
        ui.separator();

        ui.label(egui::RichText::new("Left drag: grab   Right drag: pan\nWheel: zoom").weak());
    });
    action
}
