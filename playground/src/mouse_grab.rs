use kinetica::math::Vec2;
use macroquad::prelude::*;
use kinetica::{Shape, World};

pub struct MouseGrab {
    grabbed_body: Option<usize>,
}

impl MouseGrab {
    pub fn new() -> Self {
        Self { grabbed_body: None }
    }

    pub fn update(&mut self, world: &mut World) {
        let (mouse_x, mouse_y) = mouse_position();

        if is_mouse_button_down(MouseButton::Left) {
            if self.grabbed_body.is_none() {
                // Try to grab a body
                let mouse_pos = Vec2::new(mouse_x, mouse_y);
                let grab_radius = 30.0;

                for (i, body) in world.bodies.iter().enumerate() {
                    if body.is_static() {
                        continue;
                    }

                    let dist = match body.shape {
                        Shape::Circle(r) => {
                            (body.transform.position - mouse_pos).length() - r
                        }
                        Shape::Rectangle(size) => {
                            let half = size / 2.0;
                            let min = body.transform.position - half;
                            let max = body.transform.position + half;
                            let closest = Vec2::new(
                                mouse_pos.x.clamp(min.x, max.x),
                                mouse_pos.y.clamp(min.y, max.y),
                            );
                            (mouse_pos - closest).length()
                        }
                    };

                    if dist < grab_radius {
                        self.grabbed_body = Some(i);
                        break;
                    }
                }
            }

            // Move grabbed body
            if let Some(idx) = self.grabbed_body {
                if idx < world.bodies.len() {
                    world.bodies[idx].transform.position = Vec2::new(mouse_x, mouse_y);
                    world.bodies[idx].motion.linear_velocity = Vec2::new(0.0, 0.0);
                }
            }
        } else {
            self.grabbed_body = None;
        }
    }

    pub fn draw(&self, world: &World) {
        if let Some(idx) = self.grabbed_body {
            if idx < world.bodies.len() {
                let body = &world.bodies[idx];
                draw_circle_lines(body.transform.position.x, body.transform.position.y, 5.0, 2.0, YELLOW);
            }
        }
    }
}
