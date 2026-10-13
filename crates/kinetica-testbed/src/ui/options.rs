//! The options panel: what the scene shows, how it is simulated and what is drawn.

use egui_macroquad::egui::{self, RichText};

use super::{UiAction, View};
use crate::scenes::SCENES;

const WIDTH: f32 = 270.0;
const DESCRIPTION_HEIGHT: f32 = 104.0;
const SPEEDS: [(f32, &str); 4] = [(0.1, "0.1×"), (0.25, "¼×"), (0.5, "½×"), (1.0, "1×")];
const SHORTCUTS: [(&str, &str); 9] = [
    ("Tab", "Choose a scene"),
    ("Page Up / Down", "Previous / next scene"),
    ("Space", "Pause or play"),
    ("S", "Run a single step"),
    ("R", "Restart the scene"),
    ("G  L  B", "Grab, launch, blast"),
    ("F5 / F9", "Save / restore the world"),
    ("Home", "Reset the view"),
    ("O", "Show or hide this panel"),
];

pub fn show(ctx: &egui::Context, view: &mut View) -> Option<UiAction> {
    let mut action = None;
    let frame = egui::Frame::side_top_panel(&ctx.style()).inner_margin(egui::Margin::symmetric(14, 12));
    egui::SidePanel::right("options").exact_width(WIDTH).resizable(false).frame(frame).show(ctx, |ui| {
        egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
            let scene = &SCENES[view.scene_index];
            ui.label(RichText::new(scene.category.label()).small().weak());
            ui.heading(scene.name);
            // Room for the longest description, so the options stay in place from one scene to the next.
            ui.allocate_ui(egui::vec2(ui.available_width(), DESCRIPTION_HEIGHT), |ui| {
                ui.set_min_height(DESCRIPTION_HEIGHT);
                ui.label(scene.description);
            });

            section(ui, "Simulation");
            let settings = &mut *view.settings;
            ui.add(egui::Slider::new(&mut settings.substeps, 1..=16).text("Substeps")).on_hover_text(
                "More substeps make contacts and joints stiffer: tall columns stand, chains stretch less",
            );
            ui.horizontal(|ui| {
                for (speed, label) in SPEEDS {
                    ui.selectable_value(&mut settings.speed, speed, label);
                }
                ui.label("Speed");
            });
            ui.checkbox(&mut settings.sleeping, "Sleeping").on_hover_text("Resting bodies stop being simulated");
            ui.checkbox(&mut settings.continuous, "Continuous collision")
                .on_hover_text("Fast bodies do not pass through thin ones");

            section(ui, "Display");
            let options = &mut *view.draw_options;
            ui.columns(2, |columns| {
                columns[0].checkbox(&mut options.shapes, "Shapes");
                columns[0].checkbox(&mut options.joints, "Joints");
                columns[0].checkbox(&mut options.contacts, "Contacts");
                columns[1].checkbox(&mut settings.labels, "Labels");
                columns[1].checkbox(&mut options.aabbs, "Bounds").on_hover_text("Bounding boxes of the colliders");
                columns[1].checkbox(&mut options.centers_of_mass, "Mass centers");
            });
            if ui.button("Reset view").on_hover_text("Home").clicked() {
                action = Some(UiAction::ResetView);
            }

            section(ui, "Snapshot");
            ui.horizontal(|ui| {
                if ui.button("Save").on_hover_text("F5: serialize the world to JSON").clicked() {
                    action = Some(UiAction::Save);
                }
                let restore = ui.add_enabled(view.snapshot_size.is_some(), egui::Button::new("Restore"));
                if restore.on_hover_text("F9: load the saved world back").clicked() {
                    action = Some(UiAction::Restore);
                }
                if let Some(size) = view.snapshot_size {
                    ui.label(RichText::new(format!("{:.0} kB", size as f32 / 1000.0)).weak());
                }
            });

            section(ui, "Shortcuts");
            egui::Grid::new("shortcuts").num_columns(2).spacing([12.0, 4.0]).show(ui, |ui| {
                for (key, effect) in SHORTCUTS {
                    ui.label(RichText::new(key).small());
                    ui.label(RichText::new(effect).small().weak());
                    ui.end_row();
                }
            });
        });
    });
    action
}

/// Starts a group of options with its title.
fn section(ui: &mut egui::Ui, title: &str) {
    ui.add_space(6.0);
    ui.separator();
    ui.label(RichText::new(title.to_uppercase()).small().strong());
}
