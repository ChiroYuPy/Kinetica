//! The scene picker: every scene at a glance, grouped by category, with a search field.

use egui_macroquad::egui::{self, Key, RichText, TextEdit};

use crate::scenes::{Category, SCENES, Scene};

const COLUMNS: usize = 4;
const COLUMN_WIDTH: f32 = 180.0;
/// Height of a scene in the list, and of a category title with the gap above it.
const ROW_HEIGHT: f32 = 26.0;
const TITLE_HEIGHT: f32 = 34.0;
/// Room for the longest description, so the picker does not resize as the mouse moves.
const DESCRIPTION_HEIGHT: f32 = 40.0;

#[derive(Default)]
pub struct Picker {
    open: bool,
    search: String,
}

impl Picker {
    pub fn open(&mut self) {
        self.open = true;
        self.search.clear();
    }

    /// Shows the picker if it is open, and returns the scene chosen this frame.
    pub fn show(&mut self, ctx: &egui::Context, current: usize) -> Option<usize> {
        if !self.open {
            return None;
        }
        let mut chosen = None;
        let frame = egui::Frame::popup(&ctx.style()).inner_margin(16.0);
        let modal = egui::Modal::new(egui::Id::new("scene picker")).frame(frame).show(ctx, |ui| {
            ui.set_width(COLUMNS as f32 * COLUMN_WIDTH);
            let hint = "Search a scene, Enter opens the first one found";
            let search = ui.add(TextEdit::singleline(&mut self.search).hint_text(hint).desired_width(f32::INFINITY));
            // Typing always goes to the search, the only thing here that reads the keyboard.
            search.request_focus();
            let found: Vec<usize> = (0..SCENES.len()).filter(|&index| matches(&SCENES[index], &self.search)).collect();
            // Without a search, the current scene is the one highlighted and Enter changes nothing.
            let first_found = found.first().copied().filter(|_| !self.search.is_empty());
            if ui.input(|input| input.key_pressed(Key::Enter)) {
                chosen = first_found;
            }
            let highlighted = first_found.unwrap_or(current);
            let mut described = highlighted;

            ui.scope(|ui| {
                // The height of the whole list is kept while searching, so the picker never resizes.
                ui.set_min_height(list_height());
                ui.spacing_mut().item_spacing.y = 2.0;
                ui.spacing_mut().interact_size.y = ROW_HEIGHT - 2.0;
                ui.columns(COLUMNS, |columns| {
                    for (slot, category) in Category::ALL.into_iter().enumerate() {
                        let scenes: Vec<usize> =
                            found.iter().copied().filter(|&index| SCENES[index].category == category).collect();
                        if scenes.is_empty() {
                            continue;
                        }
                        let ui = &mut columns[slot % COLUMNS];
                        ui.add_space(10.0);
                        ui.label(RichText::new(category.label()).small().weak());
                        for index in scenes {
                            let scene = ui.selectable_label(index == highlighted, SCENES[index].name);
                            if scene.hovered() {
                                described = index;
                            }
                            if scene.clicked() {
                                chosen = Some(index);
                            }
                        }
                    }
                });
                if found.is_empty() {
                    ui.label(RichText::new("No scene found").weak());
                }
            });

            ui.separator();
            egui::ScrollArea::vertical().max_height(DESCRIPTION_HEIGHT).auto_shrink([false, false]).show(ui, |ui| {
                ui.label(RichText::new(SCENES[described].description).weak());
            });
        });
        if chosen.is_some() || modal.should_close() {
            self.open = false;
        }
        chosen
    }
}

/// Height of the tallest column when every scene is listed.
fn list_height() -> f32 {
    let height = |column: usize| -> f32 {
        let categories = Category::ALL.into_iter().skip(column).step_by(COLUMNS);
        categories
            .map(|category| {
                let scenes = SCENES.iter().filter(|scene| scene.category == category).count();
                TITLE_HEIGHT + ROW_HEIGHT * scenes as f32
            })
            .sum()
    };
    (0..COLUMNS).map(height).fold(0.0, f32::max)
}

/// Whether `search` is found in the name of the scene or of its category, whatever the case.
fn matches(scene: &Scene, search: &str) -> bool {
    let search = search.trim().to_lowercase();
    scene.name.to_lowercase().contains(&search) || scene.category.label().to_lowercase().contains(&search)
}
