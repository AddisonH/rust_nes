use std::collections::HashMap;
use crate::ops;

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
// Read more here https://skilldrick.github.io/easy6502/#addressing
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
    // Indexed Indirect
    IndirectX,
    // Indirect Indexed
    IndirectY,
    Implicit,
}

// Mem reads and writes
trait Mem {
    fn mem_read(&self, addr: u16) -> u8;

    fn mem_write(&mut self, addr: u16, data: u8);
    
    fn mem_read_u16(&mut self, address: u16) -> u16 {
        u16::from_le_bytes([self.mem_read(address), self.mem_read(address + 1)])
    }

    fn mem_write_u16(&mut self, address: u16, data: u16) {
        let bytes: [u8; 2] = data.to_le_bytes();
        self.mem_write(address, bytes[0]);
        self.mem_write(address + 1, bytes[1]);
    }
}

impl Mem for CPU {
    fn mem_read(&self, addr: u16) -> u8 {
        self.memory[addr as usize]
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data;
    }
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

    fn get_operand_address(&mut self, mode: &AM) -> u16 {
        match mode {
            AM::Immediate => self.pc,
            AM::ZeroPage => self.mem_read(self.pc) as u16,
            AM::ZeroPageX => self.mem_read(self.pc).wrapping_add(self.reg_x) as u16,
            AM::ZeroPageY => self.mem_read(self.pc).wrapping_add(self.reg_y) as u16,
            AM::Absolute => self.mem_read_u16(self.pc),
            AM::AbsoluteX => self.mem_read_u16(self.pc).wrapping_add(self.reg_x as u16) as u16,
            AM::AbsoluteY => self.mem_read_u16(self.pc).wrapping_add(self.reg_y as u16) as u16,
            AM::Indirect => {
                let addr = self.mem_read_u16(self.pc);
                self.mem_read_u16(addr)
            }
            AM::IndirectX => {
                // Insane!
                // Zero page X
                let zero_page = self.mem_read(self.pc).wrapping_add(self.reg_x) as u16;
                // Indirect
                self.mem_read_u16(zero_page)
            }
            AM::IndirectY => {
                // Zero page
                let zero_page = self.mem_read(self.pc) as u16;
                // Indirect + Y
                self.mem_read_u16(zero_page).wrapping_add(self.reg_y as u16)
            }
            AM::Implicit => todo!(),
        }
    }

    pub fn run(&mut self) {
        // Reference to opcode hashmap
        let ref opcodes: HashMap<u8, &'static ops::OPS> = *ops::OPS_MAP;

        // Main loop
        loop {
            // Fetch instruction
            let opc: u8 = self.mem_read(self.pc);

            /* 
            The program counter is incremented the correct number of times based on the instruction after the match
            statement. It is incremented once here so that it correctly points to any potential operands.
            */
            self.pc += 1;

            // Store the state of the program counter to check later and avoid potential errors.
            let pc_state = self.pc;

            // Retrieve opcode information from the hashmap. Panic if the opcode is not found.
            let opcode = opcodes.get(&opc).expect(&format!("Opcode {:x} not recognized", opc));

            match opc {
                // LDA - Load Accumulator
                0xA9 | 0xA5 | 0xB5| 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
                    self.lda(&opcode.mode);
                }

                // LDX - Load X
                0xA2 | 0xA6 | 0xB6 | 0xAE | 0xBE => {
                    self.ldx(&opcode.mode);
                }

                // LDY - Load Y
                0xA0 | 0xA4 | 0xB4 | 0xAC | 0xBC => {
                    self.ldy(&opcode.mode);
                }

                // STA - Store Accumulator
                0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }

                // STX - Store X
                0x86 | 0x96 | 0x8E => {
                    self.stx(&opcode.mode);
                }

                // STY - Store Y
                0x84 | 0x94 | 0x8C => {
                    self.sty(&opcode.mode);
                }

                // TAX - Transfer Accumulator to X
                0xAA => self.tax(),

                // INX - Increment X
                0xE8 => self.inx(),

                // INY - Increment Y
                0xC8 => self.iny(),

                // BRK
                0x00 => return,

                _ => todo!(),
            }

            // Check program counter state and increment if unchanged
            if pc_state == self.pc {
                // -1 because of the prior increment
                self.pc += (opcode.bytes - 1) as u16;
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
        self.memory[0x8000..(0x8000 + program.len())].copy_from_slice(&program[..]);
        self.mem_write_u16(RESET_VECTOR, RAM_START);
    }

    pub fn load_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }

    // Begin instruction implementations
    fn lda(&mut self, mode: &AM) {
        let addr = self.get_operand_address(&mode);
        self.reg_a = self.mem_read(addr);
        self.set_zn(self.reg_a);
    }

    fn ldx(&mut self, mode: &AM) {
        let addr = self.get_operand_address(&mode);
        self.reg_x = self.mem_read(addr);
        self.set_zn(self.reg_x);
    }

    fn ldy(&mut self, mode: &AM) {
        let addr = self.get_operand_address(&mode);
        self.reg_y = self.mem_read(addr);
        self.set_zn(self.reg_y);
    }

    fn sta(&mut self, mode: &AM) {
        let addr = self.get_operand_address(&mode);
        self.mem_write(addr, self.reg_a);
    }

    fn stx(&mut self, mode: &AM) {
        let addr = self.get_operand_address(&mode);
        self.mem_write(addr, self.reg_x);
    }

    fn sty(&mut self, mode: &AM) {
        let addr = self.get_operand_address(&mode);
        self.mem_write(addr, self.reg_y);
    }

    fn tax(&mut self) {
        self.reg_x = self.reg_a;
        self.set_zn(self.reg_x);
    }

    fn inx(&mut self) {
        self.reg_x = self.reg_x.wrapping_add(1);
        self.set_zn(self.reg_x);
    }

    fn iny(&mut self) {
        self.reg_y = self.reg_y.wrapping_add(1);
        self.set_zn(self.reg_y);
    }

    // Flag helpers
    fn set_flag(&mut self, flag: u8, value: bool) {
        if value {
            self.status |= flag;
        } else {
            self.status &= !flag;
        }
    }

    // Set zero and negative flags
    fn set_zn(&mut self, result: u8) {
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
    fn test_0xa9_lda_immediate() {
        let mut cpu = CPU::new();
        cpu.load_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.reg_a, 0x05);
        assert!(cpu.status & ZERO_FLAG == 0b00);
        assert!(cpu.status & NEGATIVE_FLAG == 0);
    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.load_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status & ZERO_FLAG == 0b10);
    }

    #[test]
    fn test_0xa5_lda_zero_page() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x22);
        cpu.load_run(vec![0xa5, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x22);
    }

    #[test]
    fn test_0xad_lda_absolute() {
        let mut cpu = CPU::new();
        cpu.mem_write_u16(0x1020, 0x22);
        cpu.load_run(vec![0xad, 0x20, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x22);
    }

    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.mem_write(0x10, 0x55);
        cpu.load_run(vec![0xa5, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x55);
    }

    #[test]
    fn test_0xa1_lda_indirect_x() {
        let mut cpu = CPU::new();
        cpu.load_run(vec![0xa2, 0x01, 0xa9, 0x05, 0x85, 0x01, 0xa9, 0x07, 0x85, 0x02, 0xa0, 0x0a, 0x8c, 0x05, 0x07, 0xa1, 0x00]);
        assert_eq!(cpu.reg_a, cpu.reg_y);
    }

    #[test]
    fn test_0xb1_lda_indirect_y() {
        let mut cpu = CPU::new();
        cpu.load_run(vec![0xa0, 0x01, 0xa9, 0x03, 0x85, 0x01, 0xa9, 0x07, 0x85, 0x02, 0xa2, 0x0a, 0x8e, 0x04, 0x07, 0xb1, 0x00]);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load_run(vec![0xa9, 0x0a, 0xaa, 0x00]);

        assert_eq!(cpu.reg_x, 0x0a)
    }

    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.load_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.reg_x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.load_run(vec![0xa2, 0xff, 0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.reg_x, 1)
    }

    #[test]
    fn test_iny_overflow() {
        let mut cpu = CPU::new();
        cpu.load_run(vec![0xa0, 0xff, 0xc8, 0xc8, 0x00]);

        assert_eq!(cpu.reg_y, 1)
    }
}