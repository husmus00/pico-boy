#![allow(warnings)]

mod system;
mod registers;
mod opcodes;

use std::error::Error;
use std::rc::Rc;
use std::marker::Send;

use registers::{
    Registers,
    Reg8::*,
    Reg16::*,
    Flag::*
};

use winit::{
    window::Window,
    event::{Event, WindowEvent},
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
    error::EventLoopError,
    dpi::{LogicalSize, PhysicalSize},
};

use pixels::{
    Pixels,
    SurfaceTexture,
    wgpu::Color,
};
use crate::system::System;


fn main() {

    // Init code here
    // let mut R = Registers::default();
    // R.set16(AF, 0xF080);
    // R.set_flag(Zero, false);
    // R.set_flag(Carry, true);
    // println!("{}", R.get_flag(Zero));
    // println!("{:X}", R.get16(AF));

    // Run loop here
    // game_loop().expect("Loop failed");

    // Exit
    println!("Exiting...");
}

const GB_WIDTH: u32 = 160;
const GB_HEIGHT: u32 = 144;
const GB_SCALING_FACTOR: u32 = 2;

fn game_loop() -> Result<(), pixels::Error> {

    let event_loop = EventLoop::new().unwrap();
    let window = Window::new(&event_loop).unwrap();

    // ControlFlow::Poll continuously runs the event loop, even if the OS hasn't
    // dispatched any events. This is ideal for games and similar applications.
    event_loop.set_control_flow(ControlFlow::Poll);

    window.set_title("GameBoy - Rust");
    // Specify the size in logical dimensions like this:
    // let _ = window.request_inner_size(LogicalSize::new(GB_WIDTH, GB_HEIGHT));
    // Or specify the size in physical dimensions like this:
    let _ = window.request_inner_size(PhysicalSize::new(GB_WIDTH * GB_SCALING_FACTOR, GB_HEIGHT * GB_SCALING_FACTOR));

    // Pixels
    let size = window.inner_size();
    let surface_texture = SurfaceTexture::new(size.width, size.height, &window);

    let mut pixels = Pixels::new(GB_WIDTH, GB_HEIGHT, surface_texture)?;
    pixels.clear_color(Color::BLACK);

    // Clear the pixel buffer
    let frame = pixels.frame_mut();
    for pixel in frame.chunks_exact_mut(4) {
        pixel[0] = 0x00; // R
        pixel[1] = 0x00; // G
        pixel[2] = 0x00; // B
        pixel[3] = 0xff; // A
    }
    // frame[53000] = 0xff;
    // frame[53001] = 0xff;
    // frame[53002] = 0xff;
    // frame[53003] = 0x10;

    let mut pixel_counter = 0;

    // Draw it to the `SurfaceTexture`
    pixels.render()?;

    event_loop.run(move |event, elwt| {
        match event {
            Event::WindowEvent {
                event: WindowEvent::CloseRequested,
                ..
            } => {
                println!("The close button was pressed; stopping");
                elwt.exit();
            },
            Event::AboutToWait => {
                // Application update code.

                // Queue a RedrawRequested event.
                //
                // You only need to call this if you've determined that you need to redraw in
                // applications which do not always need to. Applications that redraw continuously
                // can render here instead.
                let frame = pixels.frame_mut();
                frame[pixel_counter] = 0xff;
                pixel_counter += 1;
                pixels.render().expect("Render failed");

                window.request_redraw();
            },
            Event::WindowEvent {
                event: WindowEvent::RedrawRequested,
                ..
            } => {
                // Redraw the application.
                //
                // It's preferable for applications that do not render continuously to render in
                // this event rather than in AboutToWait, since rendering in here allows
                // the program to gracefully handle redraws requested by the OS.
            },
            _ => ()
        }
    }).expect("Loop failed");

    return Err(pixels::Error::AdapterNotFound)
}
