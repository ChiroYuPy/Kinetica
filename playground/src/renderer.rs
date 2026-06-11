use macroquad::prelude::*;
use kinetica::{Shape, World};

pub fn render(world: &World) {
    for body in &world.bodies {
        let color = if body.is_static() {
            DARKGRAY
        } else {
            WHITE
        };

        match &body.shape {
            Shape::Circle(r) => {
                draw_circle(body.transform.position.x, body.transform.position.y, *r, color)
            }
            Shape::Rectangle(s) => {
                draw_rectangle(
                    body.transform.position.x - s.x / 2.0,
                    body.transform.position.y - s.y / 2.0,
                    s.x,
                    s.y,
                    color,
                )
            }
        }
    }

    draw_text(&format!("Bodies: {}", world.len()), 10.0, 85.0, 16.0, GRAY);
}
