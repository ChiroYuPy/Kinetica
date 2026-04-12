use kinetica::core::World;
use macroquad::prelude::*;

pub fn render(world: &World) {
    for body in &world.bodies {
        let color = if body.is_static() {
            DARKGRAY
        } else {
            WHITE
        };

        match &body.shape {
            kinetica::core::Shape::Circle(r) => {
                draw_circle(body.state.position.x, body.state.position.y, *r, color)
            }
            kinetica::core::Shape::Rectangle(s) => {
                draw_rectangle(
                    body.state.position.x - s.x / 2.0,
                    body.state.position.y - s.y / 2.0,
                    s.x,
                    s.y,
                    color,
                )
            }
        }
    }

    draw_text(&format!("Bodies: {}", world.len()), 10.0, 85.0, 16.0, GRAY);
}
