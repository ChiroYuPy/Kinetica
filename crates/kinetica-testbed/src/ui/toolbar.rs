//! The toolbar: which scene is shown, the simulation clock, and the mouse tools.

use egui_macroquad::egui::{self, Align, Button, RichText};

use super::{Interface, UiAction, View, theme};
use crate::scenes::SCENES;
use crate::tools::{Projectile, Tool};

const ICON_BUTTON_SIZE: [f32; 2] = [32.0, 26.0];
/// Wide enough for the longest scene name.
const SCENE_BUTTON_SIZE: [f32; 2] = [230.0, 26.0];

pub fn show(ctx: &egui::Context, interface: &mut Interface, view: &mut View) -> Option<UiAction> {
    let mut action = None;
    egui::TopBottomPanel::top("toolbar").exact_height(42.0).show(ctx, |ui| {
        ui.horizontal_centered(|ui| {
            if button(ui, "⏴", "Previous scene (Page Up)") {
                action = Some(UiAction::PreviousScene);
            }
            let scene = RichText::new(format!("{}  ⏷", SCENES[view.scene_index].name)).strong();
            if ui.add_sized(SCENE_BUTTON_SIZE, Button::new(scene)).on_hover_text("Choose a scene (Tab)").clicked() {
                interface.picker.open();
            }
            if button(ui, "⏵", "Next scene (Page Down)") {
                action = Some(UiAction::NextScene);
            }
            ui.separator();

            let play = RichText::new("▶").color(theme::ACCENT);
            let clicked =
                if view.paused { button(ui, play, "Play (Space)") } else { button(ui, "⏸", "Pause (Space)") };
            if clicked {
                action = Some(UiAction::TogglePause);
            }
            if button(ui, "⏭", "Run a single step (S)") {
                action = Some(UiAction::Step);
            }
            if button(ui, "⟲", "Restart the scene (R)") {
                action = Some(UiAction::Restart);
            }
            ui.separator();

            for tool in Tool::ALL {
                let hint = format!("{} ({:?})", tool.hint(), tool.key());
                ui.selectable_value(view.tool, tool, tool.label()).on_hover_text(hint);
            }
            if *view.tool == Tool::Launch {
                egui::ComboBox::from_id_salt("projectile").width(80.0).selected_text(view.projectile.label()).show_ui(
                    ui,
                    |ui| {
                        for projectile in Projectile::ALL {
                            ui.selectable_value(view.projectile, projectile, projectile.label());
                        }
                    },
                );
            }

            ui.with_layout(egui::Layout::right_to_left(Align::Center), |ui| {
                ui.toggle_value(&mut interface.options_open, "Options").on_hover_text("Show or hide the options (O)");
            });
        });
    });
    action
}

/// A button showing an icon, explaining itself when hovered; returns whether it was clicked. Its
/// size is fixed, so the toolbar does not move when the icon changes.
fn button(ui: &mut egui::Ui, icon: impl Into<egui::WidgetText>, hint: &str) -> bool {
    ui.add_sized(ICON_BUTTON_SIZE, Button::new(icon)).on_hover_text(hint).clicked()
}
