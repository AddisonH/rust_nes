use crate::bus::Bus;
use crate::ops;
use std::collections::HashMap;

// Status flags
const CARRY_FLAG: u8 = 1 << 0;
const ZERO_FLAG: u8 = 1 << 1;
const IRQ_FLAG: u8 = 1 << 2;
const DECIMAL_FLAG: u8 = 1 << 3;
const BREAK_FLAG: u8 = 1 << 4;
const BREAK_2_FLAG: u8 = 1 << 5;
const OVERFLOW_FLAG: u8 = 1 << 6;
const NEGATIVE_FLAG: u8 = 1 << 7;

const RESET_VECTOR: u16 = 0xFFFC;

const STACK: u16 = 0x0100;
const SP_INIT: u8 = 0xFD;
// https://www.nesdev.org/wiki/CPU_power_up_state

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
pub trait Mem {
    fn mem_read(&self, addr: u16) -> u8;

    fn mem_write(&mut self, addr: u16, data: u8);

    fn mem_read_u16(&self, addr: u16) -> u16 {
        u16::from_le_bytes([self.mem_read(addr), self.mem_read(addr + 1)])
    }

    fn mem_write_u16(&mut self, addr: u16, data: u16) {
        let bytes: [u8; 2] = data.to_le_bytes();
        self.mem_write(addr, bytes[0]);
        self.mem_write(addr + 1, bytes[1]);
    }
}

impl Mem for CPU {
    fn mem_read(&self, addr: u16) -> u8 {
        self.bus.mem_read(addr)
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        self.bus.mem_write(addr, data)
    }

    fn mem_read_u16(&self, addr: u16) -> u16 {
        self.bus.mem_read_u16(addr)
    }

    fn mem_write_u16(&mut self, addr: u16, data: u16) {
        self.bus.mem_write_u16(addr, data)
    }
}

// CPU struct
pub struct CPU {
    pub reg_a: u8,
    pub reg_x: u8,
    pub reg_y: u8,
    pub status: u8,
    pub pc: u16,
    pub sp: u8,
    pub bus: Bus,
}

// Begin CPU impl
impl CPU {
    pub fn new(bus: Bus) -> Self {
        CPU {
            // CPU registers
            reg_a: 0,
            reg_x: 0,
            reg_y: 0,
            // Status register
            status: 0b100100,
            // Program counter
            pc: 0,
            // Stack pointer
            sp: SP_INIT,
            // Memory 64k
            bus: bus,
        }
    }

    pub fn get_mem_addr(&self, mode: &AM, addr: u16) -> u16 {
        match mode {
            AM::Immediate => self.pc,
            AM::ZeroPage => self.mem_read(addr) as u16,
            AM::ZeroPageX => self.mem_read(addr).wrapping_add(self.reg_x) as u16,
            AM::ZeroPageY => self.mem_read(addr).wrapping_add(self.reg_y) as u16,
            AM::Absolute => self.mem_read_u16(addr),
            AM::AbsoluteX => self.mem_read_u16(addr).wrapping_add(self.reg_x as u16),
            AM::AbsoluteY => self.mem_read_u16(addr).wrapping_add(self.reg_y as u16),
            AM::Indirect => {
                let tmp = self.mem_read_u16(addr);
                self.mem_read_u16(tmp)
            }
            AM::IndirectX => {
                // Insane!
                // Zero page X
                let zero_page = self.mem_read(addr).wrapping_add(self.reg_x);

                // You cannot use mem_read_u16 here, because it will read across the page boundary
                u16::from_le_bytes([
                    self.mem_read(zero_page as u16),
                    self.mem_read(zero_page.wrapping_add(1) as u16),
                ])
            }
            AM::IndirectY => {
                // Zero page
                let zero_page = self.mem_read(addr);
                // Again, can't use mem_read_u16 here
                u16::from_le_bytes([
                    self.mem_read(zero_page as u16),
                    self.mem_read(zero_page.wrapping_add(1) as u16),
                ])
                .wrapping_add(self.reg_y as u16)
            }
            AM::Implicit => panic!("Addressing mode not supported"),
        }
    }

    fn get_op_addr(&self, mode: &AM) -> u16 {
        match mode {
            AM::Immediate => self.pc,
            _ => self.get_mem_addr(mode, self.pc),
        }
    }

    pub fn run(&mut self) {
        self.run_with_callback(|_| {});
    }

    pub fn run_with_callback<F>(&mut self, mut callback: F)
    where
        F: FnMut(&mut CPU),
    {
        // Reference to opcode hashmap
        let opcodes: &HashMap<u8, &'static ops::OPS> = &ops::OPS_MAP;

        // Main loop
        loop {
            // Execute callback before each instruction
            callback(self);

            // Fetch instruction
            let opc: u8 = self.mem_read(self.pc);
            // println!("PC: {:04X}, OP: {:02X}", self.pc, opc);

            /*
            The program counter is incremented the correct number of times based on the instruction after the match
            statement. It is incremented once here so that it correctly points to any potential operands.
            */
            self.pc += 1;

            // Store the state of the program counter to check later and avoid potential errors.
            let pc_state = self.pc;

            // Retrieve opcode information from the hashmap. Panic if the opcode is not found.
            let opcode = opcodes
                .get(&opc)
                .unwrap_or_else(|| panic!("Opcode {:x} not recognized", opc));

            match opc {
                // ADC - Add with carry
                0x69 | 0x65 | 0x75 | 0x6D | 0x7D | 0x79 | 0x61 | 0x71 => {
                    self.adc(&opcode.mode);
                }

                // AND - Logical AND
                0x29 | 0x25 | 0x35 | 0x2D | 0x3D | 0x39 | 0x21 | 0x31 => {
                    self.and(&opcode.mode);
                }

                // ASL - Arithmetic shift left
                0x06 | 0x16 | 0x0E | 0x1E => {
                    self.asl(&opcode.mode);
                }

                // ASL Accumulator
                0x0A => self.asl_acc(),

                // BCC - Branch if carry clear
                0x90 => self.bcc(),

                // BCS - Branch if carry set
                0xB0 => self.bcs(),

                // BEQ - Branch if equal
                0xF0 => self.beq(),

                // BIT - Bit test
                0x24 | 0x2C => self.bit(&opcode.mode),

                // BMI - Branch if minus
                0x30 => self.bmi(),

                // BNE - Branch if not equal
                0xD0 => self.bne(),

                // BPL - Branch if positive
                0x10 => self.bpl(),

                // BRK - Break (This will work for now)
                0x00 => return,

                // BVC - Branch if overflow clear
                0x50 => self.bvc(),

                // BVS - Branch if overflow set
                0x70 => self.bvs(),

                // CLC - Clear carry flag
                0x18 => self.clc(),

                // CLD - Clear decimal mode
                0xD8 => self.cld(),

                // CLI - Clear interrupt disable
                0x58 => self.cli(),

                // CLV - Clear overflow flag
                0xB8 => self.clv(),

                // CMP - Compare
                0xC9 | 0xC5 | 0xD5 | 0xCD | 0xDD | 0xD9 | 0xC1 | 0xD1 => {
                    self.cmp(&opcode.mode);
                }

                // CPX - Compare with X
                0xE0 | 0xE4 | 0xEC => self.cpx(&opcode.mode),

                // CPY - Compare with Y
                0xC0 | 0xC4 | 0xCC => self.cpy(&opcode.mode),

                // DEC - Decrement
                0xC6 | 0xD6 | 0xCE | 0xDE => self.dec(&opcode.mode),

                // DEX - Decrement X
                0xCA => self.dex(),

                // DEY - Decrement Y
                0x88 => self.dey(),

                // EOR - Exclusive OR
                0x49 | 0x45 | 0x55 | 0x4D | 0x5D | 0x59 | 0x41 | 0x51 => {
                    self.eor(&opcode.mode);
                }

                // INC - Increment
                0xE6 | 0xF6 | 0xEE | 0xFE => self.inc(&opcode.mode),

                // INX - Increment X
                0xE8 => self.inx(),

                // INY - Increment Y
                0xC8 => self.iny(),

                // JMP - Jump
                0x4C => self.jmp_absolute(),

                // JMP indirect
                0x6C => self.jmp_indirect(),

                // JSR - Jump to subroutine
                0x20 => self.jsr(),

                // LDA - Load Accumulator
                0xA9 | 0xA5 | 0xB5 | 0xAD | 0xBD | 0xB9 | 0xA1 | 0xB1 => {
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

                // LSR - Logical shift right
                0x46 | 0x56 | 0x4E | 0x5E => {
                    self.lsr(&opcode.mode);
                }

                // LSR Accumulator
                0x4A => self.lsr_acc(),

                // NOP - No operation
                0xEA => {}

                // ORA - Logical Inclusive OR
                0x09 | 0x05 | 0x15 | 0x0D | 0x1D | 0x19 | 0x01 | 0x11 => {
                    self.ora(&opcode.mode);
                }

                // PHA - Push Accumulator
                0x48 => self.pha(),

                // PHP - Push Processor Status
                0x08 => self.php(),

                // PLA - Pull Accumulator
                0x68 => self.pla(),

                // PLP - Pull Processor Status
                0x28 => self.plp(),

                // ROL - Rotate Left
                0x26 | 0x36 | 0x2E | 0x3E => {
                    self.rol(&opcode.mode);
                }

                // ROL Accumulator
                0x2A => self.rol_acc(),

                // ROR - Rotate Right
                0x66 | 0x76 | 0x6E | 0x7E => {
                    self.ror(&opcode.mode);
                }

                // ROR Accumulator
                0x6A => self.ror_acc(),

                // RTI - Return from Interrupt
                0x40 => self.rti(),

                // RTS - Return from Subroutine
                0x60 => self.rts(),

                // SBC - Subtract with carry
                0xE9 | 0xE5 | 0xF5 | 0xED | 0xFD | 0xF9 | 0xE1 | 0xF1 => {
                    self.sbc(&opcode.mode);
                }

                // SEC - Set carry flag
                0x38 => self.sec(),

                // SED - Set decimal mode
                0xF8 => self.sed(),

                // SEI - Set interrupt disable
                0x78 => self.sei(),

                // STA - Store Accumulator
                0x85 | 0x95 | 0x8D | 0x9D | 0x99 | 0x81 | 0x91 => {
                    self.sta(&opcode.mode);
                }

                // STX - Store X
                0x86 | 0x96 | 0x8E => self.stx(&opcode.mode),

                // STY - Store Y
                0x84 | 0x94 | 0x8C => self.sty(&opcode.mode),

                // TAX - Transfer Accumulator to X
                0xAA => self.tax(),

                // TAY - Transfer Accumulator to Y
                0xA8 => self.tay(),

                // TSX - Transfer Stack Pointer to X
                0xBA => self.tsx(),

                // TXA - Transfer X to Accumulator
                0x8A => self.txa(),

                // TXS - Transfer X to Stack Pointer
                0x9A => self.txs(),

                // TYA - Transfer Y to Accumulator
                0x98 => self.tya(),

                _ => panic!("Unknown opcode {:x}", opc),
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
        self.status = 0b100100;
        self.sp = SP_INIT;

        self.pc = self.mem_read_u16(RESET_VECTOR);
    }

    // Load program into memory and run
    pub fn load(&mut self, program: Vec<u8>) {
        for i in 0..(program.len() as u16) {
            self.mem_write(0x8600 + i, program[i as usize]);
        }
        self.mem_write_u16(RESET_VECTOR, 0x8600);
    }

    pub fn load_run(&mut self, program: Vec<u8>) {
        self.load(program);
        self.reset();
        self.run();
    }

    // Begin instruction implementations
    fn adc(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        let data = self.mem_read(addr);
        self.add_acc(data);
    }

    fn add_acc(&mut self, data: u8) {
        // Calculate sum
        let sum: u16 = self.reg_a as u16 + data as u16 + (self.status & CARRY_FLAG) as u16;

        // Set carry flag
        let carry = sum > 0xFF;
        if carry {
            self.status |= CARRY_FLAG;
        } else {
            self.clear_flag(CARRY_FLAG);
        }

        // Set overflow flag
        // https://www.righto.com/2012/12/the-6502-overflow-flag-explained.html
        let result = sum as u8;
        let overflow = (data ^ result) & (result ^ self.reg_a) & 0x80 != 0;
        if overflow {
            self.status |= OVERFLOW_FLAG;
        } else {
            self.clear_flag(OVERFLOW_FLAG);
        }

        self.reg_a = result;
        self.set_zn(self.reg_a);
    }

    fn and(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.reg_a &= self.mem_read(addr);
        self.set_zn(self.reg_a);
    }

    fn asl(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        let data = self.mem_read(addr);
        let shift_data = data.wrapping_shl(1);
        self.set_flag(CARRY_FLAG, data >> 7 == 1);
        self.mem_write(addr, shift_data);
        self.set_zn(shift_data);
    }

    fn asl_acc(&mut self) {
        self.set_flag(CARRY_FLAG, self.reg_a >> 7 == 1);
        self.reg_a = self.reg_a.wrapping_shl(1);
        self.set_zn(self.reg_a);
    }

    fn bcc(&mut self) {
        self.branch(self.status & CARRY_FLAG == 0);
    }

    fn bcs(&mut self) {
        self.branch(self.status & CARRY_FLAG != 0);
    }

    fn beq(&mut self) {
        self.branch(self.status & ZERO_FLAG != 0);
    }

    fn bit(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        let data = self.mem_read(addr);
        self.set_flag(ZERO_FLAG, data & self.reg_a == 0);
        self.set_flag(OVERFLOW_FLAG, data & 0x40 != 0);
        self.set_flag(NEGATIVE_FLAG, data & 0x80 != 0);
    }

    fn bmi(&mut self) {
        self.branch(self.status & NEGATIVE_FLAG != 0);
    }

    fn bne(&mut self) {
        self.branch(self.status & ZERO_FLAG == 0);
    }

    fn bpl(&mut self) {
        self.branch(self.status & NEGATIVE_FLAG == 0);
    }

    fn bvc(&mut self) {
        self.branch(self.status & OVERFLOW_FLAG == 0);
    }

    fn bvs(&mut self) {
        self.branch(self.status & OVERFLOW_FLAG != 0);
    }

    fn clc(&mut self) {
        self.clear_flag(CARRY_FLAG);
    }

    fn cld(&mut self) {
        self.clear_flag(DECIMAL_FLAG);
    }

    fn cli(&mut self) {
        self.clear_flag(IRQ_FLAG);
    }

    fn clv(&mut self) {
        self.clear_flag(OVERFLOW_FLAG);
    }

    fn cmp(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.set_compare(self.reg_a, self.mem_read(addr));
    }

    fn cpx(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.set_compare(self.reg_x, self.mem_read(addr));
    }

    fn cpy(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.set_compare(self.reg_y, self.mem_read(addr));
    }

    fn dec(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.mem_write(addr, self.mem_read(addr).wrapping_sub(1));
        self.set_zn(self.mem_read(addr));
    }

    fn dex(&mut self) {
        self.reg_x = self.reg_x.wrapping_sub(1);
        self.set_zn(self.reg_x);
    }

    fn dey(&mut self) {
        self.reg_y = self.reg_y.wrapping_sub(1);
        self.set_zn(self.reg_y);
    }

    fn eor(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.reg_a ^= self.mem_read(addr);
        self.set_zn(self.reg_a);
    }

    fn inc(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.mem_write(addr, self.mem_read(addr).wrapping_add(1));
        self.set_zn(self.mem_read(addr));
    }

    fn inx(&mut self) {
        self.reg_x = self.reg_x.wrapping_add(1);
        self.set_zn(self.reg_x);
    }

    fn iny(&mut self) {
        self.reg_y = self.reg_y.wrapping_add(1);
        self.set_zn(self.reg_y);
    }

    fn jmp_absolute(&mut self) {
        self.pc = self.mem_read_u16(self.pc);
    }

    fn jmp_indirect(&mut self) {
        // Special behavior for 6502 bug
        let addr = self.mem_read_u16(self.pc);
        let indirect = if addr & 0x00FF == 0x00FF {
            let lo = self.mem_read(addr);
            let hi = self.mem_read(addr & 0xFF00);
            u16::from_le_bytes([lo, hi])
        } else {
            self.mem_read_u16(addr)
        };

        self.pc = indirect;
    }

    fn jsr(&mut self) {
        let addr = self.mem_read_u16(self.pc);
        // Push return address - 1
        self.push_u16(self.pc + 2 - 1);
        self.pc = addr;
    }

    fn lda(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.reg_a = self.mem_read(addr);
        self.set_zn(self.reg_a);
    }

    fn ldx(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.reg_x = self.mem_read(addr);
        self.set_zn(self.reg_x);
    }

    fn ldy(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.reg_y = self.mem_read(addr);
        self.set_zn(self.reg_y);
    }

    fn lsr(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        let data = self.mem_read(addr);
        let shift_data = data.wrapping_shr(1);
        self.set_flag(CARRY_FLAG, data & 1 == 1);
        self.mem_write(addr, shift_data);
        self.set_zn(shift_data);
    }

    fn lsr_acc(&mut self) {
        self.set_flag(CARRY_FLAG, self.reg_a & 1 == 1);
        self.reg_a = self.reg_a.wrapping_shr(1);
        self.set_zn(self.reg_a);
    }

    fn ora(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.reg_a |= self.mem_read(addr);
        self.set_zn(self.reg_a);
    }

    fn pha(&mut self) {
        self.push(self.reg_a);
    }

    fn php(&mut self) {
        let mut flags = self.status;
        flags |= BREAK_FLAG;
        flags |= BREAK_2_FLAG;
        self.push(flags);
    }

    fn pla(&mut self) {
        self.reg_a = self.pull();
        self.set_zn(self.reg_a);
    }

    fn plp(&mut self) {
        self.status = self.pull();
        self.clear_flag(BREAK_FLAG);
        self.set_flag(BREAK_2_FLAG, true);
    }

    fn rol(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        let data = self.mem_read(addr);
        let carry_bit = self.status & CARRY_FLAG;
        let rot_data = data.wrapping_shl(1) | carry_bit;

        self.set_flag(CARRY_FLAG, data >> 7 == 1);
        self.mem_write(addr, rot_data);
        self.set_zn(rot_data);
    }

    fn rol_acc(&mut self) {
        let carry_bit = self.status & CARRY_FLAG;
        self.set_flag(CARRY_FLAG, self.reg_a >> 7 == 1);
        self.reg_a = self.reg_a.wrapping_shl(1) | carry_bit;
        self.set_zn(self.reg_a);
    }

    fn ror(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        let data = self.mem_read(addr);
        let carry_bit = self.status & CARRY_FLAG;
        let rot_data = data.wrapping_shr(1) | (carry_bit << 7);

        self.set_flag(CARRY_FLAG, data & 1 == 1);
        self.mem_write(addr, rot_data);
        self.set_zn(rot_data);
    }

    fn ror_acc(&mut self) {
        let carry_bit = self.status & CARRY_FLAG;
        self.set_flag(CARRY_FLAG, self.reg_a & 1 == 1);
        self.reg_a = self.reg_a.wrapping_shr(1) | (carry_bit << 7);
        self.set_zn(self.reg_a);
    }

    fn rti(&mut self) {
        self.status = self.pull();
        self.clear_flag(BREAK_FLAG);
        self.set_flag(BREAK_2_FLAG, true);
        self.pc = self.pull_u16();
    }

    fn rts(&mut self) {
        self.pc = self.pull_u16() + 1;
    }

    fn sbc(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        let data = self.mem_read(addr);

        // A - B = A + (-B) and -B = !B + 1
        self.add_acc(data.wrapping_neg().wrapping_sub(1));
    }

    fn sec(&mut self) {
        self.status |= CARRY_FLAG;
    }

    fn sed(&mut self) {
        self.status |= DECIMAL_FLAG;
    }

    fn sei(&mut self) {
        self.status |= IRQ_FLAG;
    }

    fn sta(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.mem_write(addr, self.reg_a);
    }

    fn stx(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.mem_write(addr, self.reg_x);
    }

    fn sty(&mut self, mode: &AM) {
        let addr = self.get_op_addr(mode);
        self.mem_write(addr, self.reg_y);
    }

    fn tax(&mut self) {
        self.reg_x = self.reg_a;
        self.set_zn(self.reg_x);
    }

    fn tay(&mut self) {
        self.reg_y = self.reg_a;
        self.set_zn(self.reg_y);
    }

    fn tsx(&mut self) {
        self.reg_x = self.sp;
        self.set_zn(self.reg_x);
    }

    fn txa(&mut self) {
        self.reg_a = self.reg_x;
        self.set_zn(self.reg_a);
    }

    fn txs(&mut self) {
        self.sp = self.reg_x;
    }

    fn tya(&mut self) {
        self.reg_a = self.reg_y;
        self.set_zn(self.reg_a);
    }

    fn push(&mut self, data: u8) {
        self.mem_write(STACK + (self.sp as u16), data);
        self.sp = self.sp.wrapping_sub(1);
    }

    fn push_u16(&mut self, data: u16) {
        // 0x1234 -> [0x34, 0x12]
        let bytes: [u8; 2] = data.to_le_bytes();
        // Push 0x12
        self.push(bytes[1]);
        // Push 0x34
        self.push(bytes[0]);
    }

    fn pull(&mut self) -> u8 {
        self.sp = self.sp.wrapping_add(1);
        self.mem_read(STACK + (self.sp as u16))
    }

    fn pull_u16(&mut self) -> u16 {
        let lo: u8 = self.pull();
        let hi = self.pull();
        u16::from_le_bytes([lo, hi])
    }

    // Flag helpers
    fn set_flag(&mut self, flag: u8, value: bool) {
        if value {
            self.status |= flag;
        } else {
            self.status &= !flag;
        }
    }

    fn clear_flag(&mut self, flag: u8) {
        self.status &= !flag;
    }

    // fn get_flag(&self, flag: u8) -> bool {
    //     (self.status & flag) != 0
    // }

    // Set zero and negative flags
    fn set_zn(&mut self, result: u8) {
        //set zero flag
        self.set_flag(ZERO_FLAG, result == 0);

        // If bit 8 == 1, set negative flag
        self.set_flag(NEGATIVE_FLAG, (result & 0x80) != 0);
    }

    // Set carry, zero, negative flags for compare instructions
    fn set_compare(&mut self, lhs: u8, rhs: u8) {
        self.set_flag(CARRY_FLAG, lhs >= rhs);
        self.set_flag(ZERO_FLAG, lhs == rhs);
        self.set_flag(NEGATIVE_FLAG, lhs.wrapping_sub(rhs) & 0x80 != 0);
    }

    fn branch(&mut self, result: bool) {
        if result {
            // bytes from the next instruction
            let jump = self.mem_read(self.pc) as i8;
            // add jump plus one
            self.pc = self.pc.wrapping_add(jump as u16).wrapping_add(1);
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::cart::test;

    #[test]
    fn test_0xa9_lda_immediate() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.reg_a, 0x05);
        assert!(cpu.status & ZERO_FLAG == 0b00);
        assert!(cpu.status & NEGATIVE_FLAG == 0);
    }

    #[test]
    fn test_0xa9_lda_zero_flag() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![0xa9, 0x00, 0x00]);
        assert!(cpu.status & ZERO_FLAG == 0b10);
    }

    #[test]
    fn test_0xa5_lda_zero_page() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.mem_write(0x10, 0x22);
        cpu.load_run(vec![0xa5, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x22);
    }

    #[test]
    fn test_0xad_lda_absolute() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.mem_write_u16(0x1020, 0x22);
        cpu.load_run(vec![0xad, 0x20, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x22);
    }

    #[test]
    fn test_0xc9_cmp_immediate_negative() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![0xa9, 0x10, 0xc9, 0x11, 0x00]);
        assert_eq!(cpu.status & NEGATIVE_FLAG, 0x80);
    }

    #[test]
    fn test_0xc9_cmp_immediate_equal() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![0xa9, 0x10, 0xc9, 0x10, 0x00]);
        assert!(cpu.status & ZERO_FLAG != 0);
    }

    #[test]
    fn test_0xc9_cmp_immediate_greater() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![0xa9, 0x11, 0xc9, 0x10, 0x00]);
        assert!(cpu.status & CARRY_FLAG != 0);
    }

    #[test]
    fn test_lda_from_memory() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.mem_write(0x10, 0x55);
        cpu.load_run(vec![0xa5, 0x10, 0x00]);
        assert_eq!(cpu.reg_a, 0x55);
    }

    #[test]
    fn test_0xa1_lda_indirect_x() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![
            0xa2, 0x01, 0xa9, 0x05, 0x85, 0x01, 0xa9, 0x07, 0x85, 0x02, 0xa0, 0x0a, 0x8c, 0x05,
            0x07, 0xa1, 0x00,
        ]);
        assert_eq!(cpu.reg_a, cpu.reg_y);
    }

    #[test]
    fn test_0xb1_lda_indirect_y() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![
            0xa0, 0x01, 0xa9, 0x03, 0x85, 0x01, 0xa9, 0x07, 0x85, 0x02, 0xa2, 0x0a, 0x8e, 0x04,
            0x07, 0xb1, 0x00,
        ]);
    }

    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![0xa9, 0x0a, 0xaa, 0x00]);

        assert_eq!(cpu.reg_x, 0x0a)
    }

    #[test]
    fn test_5_ops_working_together() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);

        assert_eq!(cpu.reg_x, 0xc1)
    }

    #[test]
    fn test_inx_overflow() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![0xa2, 0xff, 0xe8, 0xe8, 0x00]);

        assert_eq!(cpu.reg_x, 1)
    }

    #[test]
    fn test_iny_overflow() {
        let bus = Bus::new(test::test_rom());
        let mut cpu = CPU::new(bus);
        cpu.load_run(vec![0xa0, 0xff, 0xc8, 0xc8, 0x00]);

        assert_eq!(cpu.reg_y, 1)
    }
}
