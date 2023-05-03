// use std::num::Wrapping;

fn main() {
    println!("NINTENDO ENTERTAINMENT SYSTEM");
}

// Status flags
const CARRY_FLAG: u8 = 1 << 0;
const ZERO_FLAG: u8 = 1 << 1;
const IRQ_FLAG: u8 = 1 << 2;
const DECIMAL_FLAG: u8 = 1 << 3;
const BREAK_FLAG: u8 = 1 << 4;
const OVERFLOW_FLAG: u8 = 1 << 6;
const NEGATIVE_FLAG: u8 = 1 << 7;

const RESET_VECTOR: u16 = 0xFFFC;
const RAM_START: u16 = 0x8000;

// Addressing modes
#[derive(Debug)]
pub enum AM {
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Indirect,
    IndirectX,
    IndirectY,
    Implicit,
}

// CPU struct
pub struct CPU {
    pub reg_a: u8,
    pub reg_x: u8,
    pub reg_y: u8,
    pub status: u8,
    pub pc: u16,
    memory: [u8; 0xFFFF],
}

// Begin CPU impl
impl CPU {
    pub fn new() -> Self {
        CPU {
            // CPU registers
            reg_a: 0,
            reg_x: 0,
            reg_y: 0,
            // Status register
            status: 0,
            // Program counter
            pc: 0,
            // Memory 64k
            memory: [0; 0xFFFF],
        }
    }

    fn get_operand_address(&mut self, mode: AM) -> u16 {
        match mode {
            AM::Immediate => self.pc,
            AM::ZeroPage => self.mem_read(self.pc) as u16,
            AM::ZeroPageX => {
                self.mem_read(self.pc)
                .wrapping_add(self.reg_x) as u16
            }
            AM::ZeroPageY => {
                self.mem_read(self.pc)
                .wrapping_add(self.reg_y) as u16
            }
            AM::Absolute => self.mem_read_u16(self.pc),
            AM::AbsoluteX => {
                self.mem_read_u16(self.pc)
                .wrapping_add(self.reg_x as u16) as u16
            }
            AM::AbsoluteY => {
                self.mem_read_u16(self.pc)
                .wrapping_add(self.reg_y as u16) as u16
            }
            AM::Indirect => {
                let addr = self.mem_read_u16(self.pc);
                self.mem_read_u16(addr)
            },
            AM::IndirectX => {
                todo!()
            },
            AM::IndirectY => {
                todo!()
            },
            AM::Implicit => todo!(),
        }
    }

    pub fn run(&mut self) {
        loop {
            let opc: u8 = self.mem_read(self.pc);
            self.pc += 1;

            match opc {
                // LDA - Load Accumulator
                0xA9 => {
                    let param: u8 = self.mem_read(self.pc);
                    self.pc += 1;

                    self.lda(param);
                }

                // LDX - Load X
                0xA2 => {
                    let param: u8 = self.mem_read(self.pc);
                    self.pc += 1;

                    self.ldx(param);
                }

                // LDY - Load Y
                0xA0 => {
                    let param: u8 = self.mem_read(self.pc);
                    self.pc += 1;

                    self.ldy(param);
                }

                // TAX - Transfer Accumulator to X
                0xAA => self.tax(),

                0xE8 => self.inx(),

                0xC8 => self.iny(),

                // BRK
                0x00 => return,

                _ => todo!(),
            }
        }
    }

    pub fn reset(&mut self) {
        self.reg_a = 0;
        self.reg_x = 0;
        self.reg_y = 0;
        self.status = 0;

        self.pc = self.mem_read_u16(RESET_VECTOR);
    }

    // Load program into memory and run
    pub fn load(&mut self, program: Vec<u8>) {
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(RESET_VECTOR, RAM_START);
    }

    pub fn load_and_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }

    // Mem reads and writes
    fn mem_read(&self, address: u16) -> u8 {
        self.memory[address as usize]
    }

    fn mem_write(&mut self, address: u16, data: u8) {
        self.memory[address as usize] = data;
    }

    fn mem_read_u16(&mut self, address: u16) -> u16 {
        u16::from_le_bytes([self.mem_read(address), self.mem_read(address + 1)])
    }

    fn mem_write_u16(&mut self, address: u16, data: u16) {
        let bytes: [u8; 2] = data.to_le_bytes();
        self.mem_write(address, bytes[0]);
        self.mem_write(address + 1, bytes[1]);
    }

    // Begin instruction implementations
    fn lda(&mut self, value: u8) {
        self.reg_a = value;
        self.set_zero_negative_flags(self.reg_a);
    }

    fn ldx(&mut self, value: u8) {
        self.reg_x = value;
        self.set_zero_negative_flags(self.reg_x);
    }

    fn ldy(&mut self, value: u8) {
        self.reg_y = value;
        self.set_zero_negative_flags(self.reg_y);
    }

    fn tax(&mut self) {
        self.reg_x = self.reg_a;
        self.set_zero_negative_flags(self.reg_x);
    }

    fn inx(&mut self) {
        self.reg_x = self.reg_x.wrapping_add(1);
        self.set_zero_negative_flags(self.reg_x);
    }

    fn iny(&mut self) {
        self.reg_y = self.reg_y.wrapping_add(1);
        self.set_zero_negative_flags(self.reg_y);
    }

    // Flag helpers
    fn set_flag(&mut self, flag: u8, value: bool) {
        if value {
            self.status |= flag;
        } else {
            self.status &= !flag;
        }
    }

    fn set_zero_negative_flags(&mut self, result: u8) {
        //set zero flag
        self.set_flag(ZERO_FLAG, result == 0);

        // If bit 8 == 1, set negative flag
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.reg_a, 0x05);
        assert!(cpu.status & 0b0000_0010 == 0b00);
        assert!(cpu.status & 0b1000_0000 == 0);
    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status & 0b0000_0010 == 0b10);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0x0a, 0xaa, 0x00]);

        assert_eq!(cpu.reg_x, 0x0a)
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.reg_x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa2, 0xff, 0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.reg_x, 1)
    }

    #[test]
    fn test_iny_overflow() {
        let mut cpu = CPU::new();
        cpu.load_and_run(vec![0xa0, 0xff, 0xc8, 0xc8, 0x00]);

        assert_eq!(cpu.reg_y, 1)
    }
}
