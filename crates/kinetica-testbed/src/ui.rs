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
}

/// What the panel shows, and the display options it edits in place.
pub struct PanelState<'a> {
    pub scene_index: usize,
    pub paused: bool,
    pub time: f32,
    pub stats: Stats,
    pub draw_options: &'a mut DebugDrawOptions,
}

pub struct Stats {
    pub bodies: usize,
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

        ui.label("Show");
        let options = &mut *state.draw_options;
        ui.checkbox(&mut options.shapes, "Shapes");
        ui.checkbox(&mut options.joints, "Joints");
        ui.checkbox(&mut options.contacts, "Contacts");
        ui.checkbox(&mut options.aabbs, "Bounding boxes");
        ui.checkbox(&mut options.centers_of_mass, "Centers of mass");
        ui.separator();

        let stats = &state.stats;
        ui.label(format!("Bodies: {}   Joints: {}", stats.bodies, stats.joints));
        ui.label(format!("Contacts: {}", stats.contacts));
        ui.label(format!("Step: {:.3} ms   {} fps", stats.step_ms, stats.fps));
        ui.separator();

        ui.label(egui::RichText::new("Left drag: grab   Right drag: pan\nWheel: zoom").weak());
    });
    action
}
