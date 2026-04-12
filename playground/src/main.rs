use macroquad::prelude::*;

#[macroquad::main("Kinetica Physics Engine")]
async fn main() {
    loop {
        clear_background(Color::new(0.125, 0.125, 0.125, 1.0));

        next_frame().await
    }
}