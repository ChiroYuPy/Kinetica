//! The look of the interface: dark, flat and quiet, with one warm accent.

use egui_macroquad::egui::{self, Color32, CornerRadius, FontId, Shadow, Stroke, TextStyle};

pub const PANEL: Color32 = Color32::from_rgb(27, 30, 36);
const BORDER: Color32 = Color32::from_rgb(44, 48, 57);
const FIELD: Color32 = Color32::from_rgb(19, 21, 25);
const WIDGET: Color32 = Color32::from_rgb(40, 44, 52);
const WIDGET_HOVERED: Color32 = Color32::from_rgb(52, 57, 67);
const WIDGET_PRESSED: Color32 = Color32::from_rgb(62, 68, 80);
const TEXT: Color32 = Color32::from_rgb(196, 201, 209);
const STRONG_TEXT: Color32 = Color32::from_rgb(238, 240, 243);
/// The orange of dynamic bodies.
pub const ACCENT: Color32 = Color32::from_rgb(242, 166, 77);
const ACCENT_BACKGROUND: Color32 = Color32::from_rgb(78, 60, 40);
const CORNER_RADIUS: CornerRadius = CornerRadius::same(6);

/// Styles every egui widget.
pub fn apply(ctx: &egui::Context) {
    ctx.style_mut(|style| {
        style.text_styles = [
            (TextStyle::Small, FontId::proportional(12.0)),
            (TextStyle::Body, FontId::proportional(14.0)),
            (TextStyle::Button, FontId::proportional(14.0)),
            (TextStyle::Heading, FontId::proportional(18.0)),
            (TextStyle::Monospace, FontId::monospace(13.0)),
        ]
        .into();
        style.spacing.item_spacing = egui::vec2(8.0, 6.0);
        style.spacing.button_padding = egui::vec2(9.0, 4.0);
        style.spacing.interact_size.y = 26.0;
        style.spacing.slider_width = 120.0;

        let visuals = &mut style.visuals;
        visuals.panel_fill = PANEL;
        visuals.window_fill = PANEL;
        visuals.window_stroke = Stroke::new(1.0_f32, BORDER);
        visuals.window_corner_radius = CornerRadius::same(10);
        visuals.menu_corner_radius = CORNER_RADIUS;
        let shadow = Shadow { offset: [0, 8], blur: 24, spread: 0, color: Color32::from_black_alpha(110) };
        visuals.window_shadow = shadow;
        visuals.popup_shadow = shadow;
        visuals.extreme_bg_color = FIELD;
        visuals.selection.bg_fill = ACCENT_BACKGROUND;
        visuals.selection.stroke = Stroke::new(1.0_f32, ACCENT);
        visuals.slider_trailing_fill = true;

        let widgets = &mut visuals.widgets;
        widgets.noninteractive.bg_fill = PANEL;
        widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, BORDER);
        widgets.noninteractive.fg_stroke = Stroke::new(1.0_f32, TEXT);
        for (widget, fill, text) in [
            (&mut widgets.inactive, WIDGET, TEXT),
            (&mut widgets.open, WIDGET, TEXT),
            (&mut widgets.hovered, WIDGET_HOVERED, STRONG_TEXT),
            (&mut widgets.active, WIDGET_PRESSED, STRONG_TEXT),
        ] {
            widget.bg_fill = fill;
            widget.weak_bg_fill = fill;
            widget.bg_stroke = Stroke::NONE;
            widget.fg_stroke = Stroke::new(1.0_f32, text);
            widget.corner_radius = CORNER_RADIUS;
            widget.expansion = 0.0;
        }
        widgets.noninteractive.corner_radius = CORNER_RADIUS;
    });
}
