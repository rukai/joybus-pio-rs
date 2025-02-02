#![no_std]

//! An implementation of the controller side of the joybus protocol for gamecube for the RP2040 chip via its PIO functionality.
//!
//! joybus-pio and haybox were heavily referenced in the implementation.
//!
//! For understanding how the inner protocol works consider
//! [this excellent writeup on the GC controller protocol](https://jefflongo.dev/posts/gc-controller-reverse-engineering-part-1)

use cortex_m::delay::Delay;
use embedded_hal::digital::InputPin;
use pio::{Instruction, InstructionOperands, Program, ProgramWithDefines, SideSet, Wrap};
use rp2040_hal::{
    clocks::Clock,
    clocks::ClocksManager,
    gpio::{bank0::Gpio28, FunctionNull, FunctionPio0, Pin, PullDown},
    pac::{PIO0, RESETS},
    pio::{PIOExt, Running, Rx, ShiftDirection, StateMachine, Tx, SM0},
    Timer,
};

/// A wrapper around the PIO types from the rp2040 HAL required for low level communication over the joybus protocol.
pub struct JoybusPio {
    data_pin: Pin<Gpio28, FunctionPio0, PullDown>,
    tx: Tx<(PIO0, SM0)>,
    rx: Rx<(PIO0, SM0)>,
    sm: StateMachine<(PIO0, SM0), Running>,
}

impl JoybusPio {
    pub fn new(
        data_pin: Pin<Gpio28, FunctionNull, PullDown>,
        pio0: PIO0,
        resets: &mut RESETS,
        clocks: ClocksManager,
    ) -> JoybusPio {
        let data_pin: Pin<_, FunctionPio0, PullDown> = data_pin.into_function();
        let data_pin_num = data_pin.id().num;

        //     let program = pio_proc::pio_asm!(
        //         "
        // .define public T1 10
        // .define public T2 20
        // .define public T3 10

        // ; Autopush with 8 bit ISR threshold
        // public read:
        //     set pindirs 0                   ; Set pin to input
        // read_loop:
        //     wait 0 pin 0 [T1 + T2 / 2 - 1]  ; Wait for falling edge, then wait until halfway through the 2uS which represents the bit value
        //     in pins, 1                      ; Read bit value
        //     wait 1 pin 0                    ; Done reading, so make sure we wait for the line to go high again before restarting the loop
        //     jmp read_loop

        // ; 9 bit OSR threshold, no autopull because it interferes with !osre
        // public write:
        //     set pindirs 1           ; Set pin to output
        // write_loop:
        //     set pins, 1             ; Set line high for at least 1uS to end pulse
        //     pull ifempty block      ; Fetch next byte into OSR if we are done with the current one
        //     out x, 1                ; Get bit
        //     jmp !osre write_bit     ; If we aren't on the 9th bit, just write the bit
        //     jmp x!=y write_stop_bit ; If we are on the 9th bit and it's a 1 that indicates stop bit so write it
        //     pull ifempty block      ; If we are on the 9th bit and it's a 0 then we should skip to the next byte
        //     out x, 1                ; Get first bit of the next byte
        //     jmp write_bit_fast      ; Write it, skipping some of the delays because we spent so much time checking the 9th bit
        // write_bit:
        //     nop [3]
        // write_bit_fast:
        //     nop [T3 - 9]
        //     set pins, 0 [T1 - 1]    ; Pulse always starts with low for 1uS
        //     mov pins, x [T2 - 2]    ; Set line according to bit value for 2uS
        //     jmp write_loop
        // write_stop_bit:
        //     nop [T3 - 6]
        //     set pins, 0 [T1 - 1]
        //     set pins, 1 [T2 - 2]
        //     jmp read
        // "
        //     );

        // pio proc macro is broken with cargo bin deps nightly feature.
        // work around this by manually creating program.
        let raw_program: [u16; 32] = [
            //     .wrap_target
            0xe080, //  0: set    pindirs, 0
            0x3320, //  1: wait   0 pin, 0               [19]
            0x4001, //  2: in     pins, 1
            0x20a0, //  3: wait   1 pin, 0
            0x0001, //  4: jmp    1
            0xe081, //  5: set    pindirs, 1
            0xe001, //  6: set    pins, 1
            0x80e0, //  7: pull   ifempty block
            0x6021, //  8: out    x, 1
            0x00ee, //  9: jmp    !osre, 14
            0x00b3, // 10: jmp    x != y, 19
            0x80e0, // 11: pull   ifempty block
            0x6021, // 12: out    x, 1
            0x000f, // 13: jmp    15
            0xa342, // 14: nop                           [3]
            0xa142, // 15: nop                           [1]
            0xe900, // 16: set    pins, 0                [9]
            0xb201, // 17: mov    pins, x                [18]
            0x0006, // 18: jmp    6
            0xa442, // 19: nop                           [4]
            0xe900, // 20: set    pins, 0                [9]
            0xf201, // 21: set    pins, 1                [18]
            0x0000, // 22: jmp    0
            //     .wrap
            0x0000, // padding
            0x0000, // padding
            0x0000, // padding
            0x0000, // padding
            0x0000, // padding
            0x0000, // padding
            0x0000, // padding
            0x0000, // padding
            0x0000, // padding
        ];

        let program = ProgramWithDefines {
            program: Program {
                code: raw_program.into(),
                origin: Some(0),
                wrap: Wrap {
                    source: 22,
                    target: 0,
                },
                side_set: SideSet::default(),
            },
            public_defines: (),
        };

        let (mut pio, sm0, _, _, _) = pio0.split(resets);
        let installed = pio
        .install(&program.program)
        .unwrap()
        // TODO: do we need this or does rp2040_hal derive it for us?
        //.set_wrap()
        ;

        // TODO: this math is a direct port from joybus-pio.
        //       but with the non-deprecated clock_divisor_fixed_point method the math looks weird but is still equivalent.
        //       If I can print the values with a debugger I could probably understand it well enough to simplify.
        let bitrate = 250000;
        let cycles_per_bit = 10 + 20 + 10;
        let divisor = clocks.system_clock.freq().to_Hz() as f32 / (cycles_per_bit * bitrate) as f32;

        let (sm, rx, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            .out_pins(data_pin_num, 1)
            .set_pins(data_pin_num, 1)
            .in_pin_base(data_pin_num)
            // out shift
            .out_shift_direction(ShiftDirection::Left)
            .autopull(false)
            .pull_threshold(9)
            // in shift
            .in_shift_direction(ShiftDirection::Left)
            .autopush(true)
            .push_threshold(8)
            .clock_divisor_fixed_point(divisor as u16, (divisor * 256.0) as u8)
            .build(sm0);
        let sm = sm.start();

        JoybusPio {
            tx,
            rx,
            sm,
            data_pin,
        }
    }
}

/// A wrapper around [`JoybusPio`] providing a high level interface for acting as a gamecube controller.
pub struct GamecubeController {
    pio: JoybusPio,
}

impl GamecubeController {
    /// Initializes a connection with a gamecube protocol compatible device and
    /// returns a [`GamecubeController`] instance to interact with this connection.
    /// If None is returned the device is not compatible with the gamecube protocol.
    pub fn try_new(
        mut pio: JoybusPio,
        timer: &Timer,
        delay: &mut Delay,
    ) -> Option<GamecubeController> {
        pio.sm.exec_instruction(Instruction {
            operands: InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: 0,
            },
            delay: 0,
            side_set: None,
        });

        let mut controller = GamecubeController { pio };

        match controller.recv(timer).map(GamecubeCommand::from) {
            Some(GamecubeCommand::Reset) | Some(GamecubeCommand::Probe) => {
                delay.delay_us(4);
                controller.send(&[9, 0, 3]);
            }
            Some(GamecubeCommand::Recalibrate) | Some(GamecubeCommand::Origin) => {
                delay.delay_us(4);
                // set perfect deadzone, we have no analog sticks
                // Apparently gc adapter ignores this though and uses the first poll response instead.
                controller.send(&[
                    0,           // butons1
                    0b1000_0000, // butons2
                    128,         // stick x
                    128,         // stick y
                    128,         // cstick x
                    128,         // cstick y
                    0,           // left trigger
                    0,           // right trigger
                    0,           // reserved
                    0,           // reserved
                ]);
            }
            Some(GamecubeCommand::Poll) => {
                let report = [
                    0,           // butons1
                    0b1000_0000, // butons2
                    128,         // stick x
                    128,         // stick y
                    128,         // cstick x
                    128,         // cstick y
                    0,           // left trigger
                    0,           // right trigger
                ];
                controller.respond_to_poll_raw(timer, delay, &report);
            }
            Some(GamecubeCommand::Unknown) => {
                delay.delay_us(130);
                controller.restart_sm_for_read();
            }
            None => return None,
        }

        Some(controller)
    }

    pub fn wait_for_poll_start(&mut self, timer: &Timer, delay: &mut Delay) {
        loop {
            match self.recv(timer).map(GamecubeCommand::from) {
                Some(GamecubeCommand::Reset) | Some(GamecubeCommand::Probe) => {
                    delay.delay_us(4);
                    self.send(&[9, 0, 3]);
                }
                Some(GamecubeCommand::Recalibrate) | Some(GamecubeCommand::Origin) => {
                    delay.delay_us(4);
                    // set perfect deadzone, we have no analog sticks
                    // Apparently gc adapter ignores this though and uses the first poll response instead.
                    self.send(&[
                        0,   // butons1
                        1,   // butons2
                        128, // stick x
                        128, // stick y
                        128, // cstick x
                        128, // cstick y
                        0,   // left trigger
                        0,   // right trigger
                        0,   // reserved
                        0,   // reserved
                    ]);
                }
                Some(GamecubeCommand::Poll) => {
                    return;
                }
                Some(GamecubeCommand::Unknown) | None => {
                    delay.delay_us(130);
                    self.restart_sm_for_read();
                }
            }
        }
    }

    pub fn restart_sm_for_read(&mut self) {
        self.pio.sm.clear_fifos(); // TODO: this should probably occur inside the restart
        self.pio.sm.restart();
    }

    pub fn restart_sm_for_write(&mut self) {
        self.pio.sm.clear_fifos(); // TODO: this should probably occur inside the restart
        self.pio.sm.restart();
        self.pio.sm.exec_instruction(Instruction {
            operands: InstructionOperands::JMP {
                condition: pio::JmpCondition::Always,
                address: 5,
            },
            delay: 0,
            side_set: None,
        });
    }

    pub fn respond_to_poll(&mut self, timer: &Timer, delay: &mut Delay, input: GamecubeInput) {
        self.respond_to_poll_raw(timer, delay, &input.create_report());
    }

    pub fn respond_to_poll_raw(&mut self, timer: &Timer, delay: &mut Delay, report: &[u8]) {
        delay.delay_us(40);

        self.recv(timer);
        self.recv(timer);
        delay.delay_us(4);

        self.send(report);
    }

    pub fn recv(&mut self, timer: &Timer) -> Option<u8> {
        let instant = timer.get_counter();

        loop {
            match self.pio.rx.read() {
                Some(value) => return Some(value as u8),
                None => {
                    if timer
                        .get_counter()
                        .checked_duration_since(instant)
                        .unwrap()
                        .ticks()
                        // TODO: high value used for testing
                        > 2000000
                    {
                        return None;
                    }
                }
            }
        }
    }

    pub fn send(&mut self, values: &[u8]) {
        // wait for line to be high
        while self.pio.data_pin.as_input().is_low().unwrap() {}

        self.restart_sm_for_write();

        for (i, value) in values.iter().enumerate() {
            let stop = if i == values.len() - 1 { 1 } else { 0 };
            let word = ((*value as u32) << 24) | ((stop as u32) << 23);

            while self.pio.tx.is_full() {}
            self.pio.tx.write(word);
        }
    }
}

enum GamecubeCommand {
    Probe = 0x00,
    Poll = 0x40,
    Origin = 0x41,
    Recalibrate = 0x42,
    Reset = 0xFF,
    Unknown,
}

impl GamecubeCommand {
    fn from(value: u8) -> Self {
        match value {
            0x00 => GamecubeCommand::Probe,
            0xFF => GamecubeCommand::Reset,
            0x41 => GamecubeCommand::Origin,
            0x42 => GamecubeCommand::Recalibrate,
            0x40 => GamecubeCommand::Poll,
            _ => GamecubeCommand::Unknown,
        }
    }
}

/// Specify the button and stick inputs to be provided to a gamecube compatible device.
pub struct GamecubeInput {
    pub start: bool,
    pub a: bool,
    pub b: bool,
    pub x: bool,
    pub y: bool,
    pub z: bool,
    pub dpad_up: bool,
    pub dpad_down: bool,
    pub dpad_left: bool,
    pub dpad_right: bool,
    pub l_digital: bool,
    pub r_digital: bool,
    pub stick_x: u8,
    pub stick_y: u8,
    pub cstick_x: u8,
    pub cstick_y: u8,
    pub l_analog: u8,
    pub r_analog: u8,
}

impl GamecubeInput {
    fn create_report(&self) -> [u8; 8] {
        #[rustfmt::skip]
        let buttons1 =
              if self.a     { 0b0000_0001 } else { 0 }
            | if self.b     { 0b0000_0010 } else { 0 }
            | if self.x     { 0b0000_0100 } else { 0 }
            | if self.y     { 0b0000_1000 } else { 0 }
            | if self.start { 0b0001_0000 } else { 0 };

        #[rustfmt::skip]
        let buttons2 = 0b1000_0000
            | if self.dpad_left  { 0b0000_0001 } else { 0 }
            | if self.dpad_right { 0b0000_0010 } else { 0 }
            | if self.dpad_down  { 0b0000_0100 } else { 0 }
            | if self.dpad_up    { 0b0000_1000 } else { 0 }
            | if self.z          { 0b0001_0000 } else { 0 }
            | if self.l_digital  { 0b0010_0000 } else { 0 }
            | if self.r_digital  { 0b0100_0000 } else { 0 };

        [
            buttons1,
            buttons2,
            self.stick_x,
            self.stick_y,
            self.cstick_x,
            self.cstick_y,
            self.l_analog,
            self.r_analog,
        ]
    }
}
