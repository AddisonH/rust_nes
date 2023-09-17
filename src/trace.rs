use crate::cpu::Mem;
use crate::cpu::AM;
use crate::cpu::CPU;
use crate::ops;
use std::collections::HashMap;

pub fn trace(cpu: &CPU) -> String {
    let opcodes: &HashMap<u8, &'static ops::OPS> = &ops::OPS_MAP;

    let pc = cpu.pc;
    let opcode = cpu.mem_read(pc);
    let ops_struct = opcodes.get(&opcode).unwrap();
    // Vector containing 1-3 bytes of the instruction
    let mut ihex = vec![];
    ihex.push(opcode);

    let (mem_addr, stored_value) = match ops_struct.mode {
        AM::Immediate | AM::Implicit => (0, 0),
        _ => {
            let addr = cpu.get_mem_addr(&ops_struct.mode, pc + 1);
            (addr, cpu.mem_read(addr))
        }
    };

    // List of things to do
    // 1. Print the program counter
    // 2. Print the instruction bytes
    // 3. Print the assembly
    // 4. Print the address thingy
    // 5. Print the CPU registers

    let addr_string = match ops_struct.bytes {
        1 => match ops_struct.code {
            // Special case for shift and rotate instructions. Need to have an "A" in opcode column.
            0x0a | 0x4a | 0x2a | 0x6a => format!("A "),
            _ => String::from(""),
        },
        2 => {
            // Get second byte
            let addr = cpu.mem_read(pc + 1);
            ihex.push(addr);

            match ops_struct.mode {
                // Ex: CMP #$7F
                AM::Immediate => format!("#${:02x}", addr),

                // Ex: BIT $01 = FF
                AM::ZeroPage => format!("${:02x} = {:02x}", mem_addr, stored_value),

                // Ex: LDY $33,X @ 33 = AA
                AM::ZeroPageX => {
                    format!("${:02x},X @ {:02x} = {:02x}", addr, mem_addr, stored_value)
                }

                // Ex: LDX $00,Y @ 78 = 33
                AM::ZeroPageY => {
                    format!("${:02x},Y @ {:02x} = {:02x}", addr, mem_addr, stored_value)
                }

                // Ex: LDA ($80,X) @ 80 = 0200 = 5A
                AM::IndirectX => format!(
                    "(${:02x},X) @ {:02x} = {:04x} = {:02x}",
                    addr,
                    (addr.wrapping_add(cpu.reg_x)),
                    mem_addr,
                    stored_value
                ),

                // Ex: ADC ($33),Y = 0400 @ 0400 = 69
                AM::IndirectY => format!(
                    "(${:02x}),Y = {:04x} @ {:04x} = {:02x}",
                    addr,
                    (mem_addr.wrapping_sub(cpu.reg_y as u16)),
                    mem_addr,
                    stored_value
                ),

                // Ex: BNE $C771
                AM::Implicit => {
                    // This be for instructions that do local jumps
                    let jump_addr: usize = (pc as usize + 2).wrapping_add((addr as i8) as usize);
                    format!("${:04x}", jump_addr)
                }
                _ => panic!("Shouldn't be here"),
            }
        }
        3 => {
            ihex.push(cpu.mem_read(pc + 1));
            ihex.push(cpu.mem_read(pc + 2));

            let addr = cpu.mem_read_u16(pc + 1);

            match ops_struct.mode {
                AM::Implicit => format!("${:04x}", addr),
                AM::Absolute => format!("${:04x} = {:02x}", mem_addr, stored_value),
                AM::AbsoluteX => {
                    format!("${:04x},X @ {:04x} = {:02x}", addr, mem_addr, stored_value)
                }
                AM::AbsoluteY => {
                    format!("${:04x},Y @ {:04x} = {:02x}", addr, mem_addr, stored_value)
                }

                // Special code for JMP indirect
                AM::Indirect => {
                    let indirect = if addr & 0x00FF == 0x00FF {
                        let lo = cpu.mem_read(addr);
                        let hi = cpu.mem_read(addr & 0xFF00);
                        u16::from_le_bytes([lo, hi])
                    } else {
                        cpu.mem_read_u16(addr)
                    };
                    format!("(${:04x}) = {:04x}", addr, indirect)
                }
                _ => panic!("Shouldn't be here"),
            }
        }
        _ => panic!("Shouldn't be here"),
    };

    let hex_str = ihex
        .iter()
        .map(|z| format!("{:02x}", z))
        .collect::<Vec<String>>()
        .join(" ");

    format!(
        "{:04x}  {:8}  {:3} {:26}  A:{:02x} X:{:02x} Y:{:02x} P:{:02x} SP:{:02x}",
        pc,
        hex_str,
        ops_struct.name,
        addr_string,
        cpu.reg_a,
        cpu.reg_x,
        cpu.reg_y,
        cpu.status,
        cpu.sp
    )
    .to_ascii_uppercase()
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::bus::Bus;
    use crate::cart::test::test_rom;

    #[test]
    fn test_format_trace() {
        let mut bus = Bus::new(test_rom());
        bus.mem_write(100, 0xa2);
        bus.mem_write(101, 0x01);
        bus.mem_write(102, 0xca);
        bus.mem_write(103, 0x88);
        bus.mem_write(104, 0x00);

        let mut cpu = CPU::new(bus);
        cpu.pc = 0x64;
        cpu.reg_a = 1;
        cpu.reg_x = 2;
        cpu.reg_y = 3;
        let mut result: Vec<String> = vec![];
        cpu.run_with_callback(|cpu| {
            result.push(trace(cpu));
        });
        assert_eq!(
            "0064  A2 01     LDX #$01                        A:01 X:02 Y:03 P:24 SP:FD",
            result[0]
        );
        assert_eq!(
            "0066  CA        DEX                             A:01 X:01 Y:03 P:24 SP:FD",
            result[1]
        );
        assert_eq!(
            "0067  88        DEY                             A:01 X:00 Y:03 P:26 SP:FD",
            result[2]
        );
    }

    #[test]
    fn test_format_mem_access() {
        let mut bus = Bus::new(test_rom());
        // ORA ($33), Y
        bus.mem_write(100, 0x11);
        bus.mem_write(101, 0x33);

        //data
        bus.mem_write(0x33, 00);
        bus.mem_write(0x34, 04);

        //target cell
        bus.mem_write(0x400, 0xAA);

        let mut cpu = CPU::new(bus);
        cpu.pc = 0x64;
        cpu.reg_y = 0;
        let mut result: Vec<String> = vec![];
        cpu.run_with_callback(|cpu| {
            result.push(trace(cpu));
        });
        assert_eq!(
            "0064  11 33     ORA ($33),Y = 0400 @ 0400 = AA  A:00 X:00 Y:00 P:24 SP:FD",
            result[0]
        );
    }
}
