pub mod bus;
pub mod cart;
pub mod cpu;
pub mod ops;
pub mod trace;

use bus::Bus;
use cart::Rom;
use cpu::Mem;
use cpu::CPU;
use trace::trace;

use rand::Rng;
use sdl2::event::Event;
use sdl2::keyboard::Keycode;
use sdl2::pixels::Color;
use sdl2::pixels::PixelFormatEnum;
use sdl2::EventPump;

#[macro_use]
extern crate lazy_static;

// Handle keyboard input
// fn handle_user_input(cpu: &mut CPU, event_pump: &mut EventPump) {
//     for event in event_pump.poll_iter() {
//         match event {
//             Event::Quit { .. }
//             | Event::KeyDown {
//                 keycode: Some(Keycode::Escape),
//                 ..
//             } => {
//                 std::process::exit(0);
//             }
//             Event::KeyDown {
//                 keycode: Some(Keycode::W),
//                 ..
//             } => {
//                 cpu.mem_write(0xff, 0x77);
//             }
//             Event::KeyDown {
//                 keycode: Some(Keycode::S),
//                 ..
//             } => {
//                 cpu.mem_write(0xff, 0x73);
//             }
//             Event::KeyDown {
//                 keycode: Some(Keycode::A),
//                 ..
//             } => {
//                 cpu.mem_write(0xff, 0x61);
//             }
//             Event::KeyDown {
//                 keycode: Some(Keycode::D),
//                 ..
//             } => {
//                 cpu.mem_write(0xff, 0x64);
//             }
//             _ => { /* nothing */ }
//         }
//     }
// }

// Byte color codes
// fn color(byte: u8) -> Color {
//     match byte {
//         0 => sdl2::pixels::Color::BLACK,
//         1 => sdl2::pixels::Color::WHITE,
//         2 | 9 => sdl2::pixels::Color::GREY,
//         3 | 10 => sdl2::pixels::Color::RED,
//         4 | 11 => sdl2::pixels::Color::GREEN,
//         5 | 12 => sdl2::pixels::Color::BLUE,
//         6 | 13 => sdl2::pixels::Color::MAGENTA,
//         7 | 14 => sdl2::pixels::Color::YELLOW,
//         _ => sdl2::pixels::Color::CYAN,
//     }
// }

// This is so the screen doesn't update every instruction
// fn read_screen_state(cpu: &CPU, frame: &mut [u8; 32 * 32 * 3]) -> bool {
//     let mut frame_index = 0;
//     let mut update_screen = false;

//     // Iterate over screen memory space
//     for i in 0x0200..0x0600 {
//         let color_index = cpu.mem_read(i as u16);
//         let (r, g, b) = color(color_index).rgb();
//         if frame[frame_index] != r || frame[frame_index + 1] != g || frame[frame_index + 2] != b {
//             frame[frame_index] = r;
//             frame[frame_index + 1] = g;
//             frame[frame_index + 2] = b;
//             update_screen = true;
//         }
//         frame_index += 3;
//     }
//     update_screen
// }

fn main() {
    // Setup SDL stuff
    // let sdl_context = sdl2::init().unwrap();
    // let video_subsystem = sdl_context.video().unwrap();
    // let window = video_subsystem
    //     .window("Snake game", (32.0 * 10.0) as u32, (32.0 * 10.0) as u32)
    //     .position_centered()
    //     .build()
    //     .unwrap();

    // let mut canvas = window.into_canvas().present_vsync().build().unwrap();
    // let mut event_pump = sdl_context.event_pump().unwrap();
    // canvas.set_scale(10.0, 10.0).unwrap();

    // let creator = canvas.texture_creator();
    // let mut texture = creator
    //     .create_texture_target(PixelFormatEnum::RGB24, 32, 32)
    //     .unwrap();

    // Create cpu and load program
    let raw: Vec<u8> = std::fs::read("nestest.nes").unwrap();
    let rom = Rom::new(&raw).unwrap();
    let bus = Bus::new(rom);

    let mut cpu = CPU::new(bus);
    cpu.reset();
    cpu.pc = 0xC000;

    // Init screen state
    // let mut screen_state = [0_u8; 32 * 32 * 3];
    // let mut rng = rand::thread_rng();

    // Start cpu with input handling and screen updating callback
    cpu.run_with_callback(move |cpu| {
        println!("{}", trace(cpu));
        // handle_user_input(cpu, &mut event_pump);
        // cpu.mem_write(0xfe, rng.gen_range(1, 16));

        // if read_screen_state(cpu, &mut screen_state) {
        //     texture.update(None, &screen_state, 32 * 3).unwrap();
        //     canvas.copy(&texture, None, None).unwrap();
        //     canvas.present();
        // }

        // // Sleep so it doesn't run so darn fast
        // ::std::thread::sleep(::std::time::Duration::new(0, 70_000));
    });
}
