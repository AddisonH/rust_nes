use crate::cart::Rom;
use crate::cpu::Mem;

const RAM_START: u16 = 0x0000;
const RAM_END: u16 = 0x1FFF;
const PPU_START: u16 = 0x2000;
const PPU_END: u16 = 0x3FFF;
const ROM_START: u16 = 0x8000;
const ROM_END: u16 = 0xFFFF;

const RAM_MASK: u16 = 0x07FF;
const PPU_MASK: u16 = 0x2007;

pub struct Bus {
    // 2k ram
    pub cpu_ram: [u8; 2048],
    rom: Rom,
}

impl Bus {
    pub fn new(rom: Rom) -> Bus {
        Bus {
            cpu_ram: [0; 2048],
            rom: rom,
        }
    }

    fn read_prg_rom(&self, mut addr: u16) -> u8 {
        addr -= ROM_START;
        if self.rom.prg_rom.len() == 0x4000 && addr >= 0x4000 {
            addr = addr % 0x4000;
        }
        self.rom.prg_rom[addr as usize]
    }
}

/* CPU has 2k RAM (11 bits addressing) with 8k addressing space (13 bits addressing)
 * We have to mask out the upper two bits to get the correct address (mirroring)
 * PPU is the same idea.
 */
impl Mem for Bus {
    fn mem_read(&self, addr: u16) -> u8 {
        match addr {
            RAM_START..=RAM_END => {
                let mirror_addr = addr & RAM_MASK;
                self.cpu_ram[mirror_addr as usize]
            }

            PPU_START..=PPU_END => {
                let _ = addr & PPU_MASK;
                todo!("No PPU support yet")
            }

            ROM_START..=ROM_END => self.read_prg_rom(addr),

            _ => {
                println!("Read from unmapped address: 0x{:04X}", addr);
                0
            }
        }
    }

    fn mem_write(&mut self, addr: u16, data: u8) {
        match addr {
            RAM_START..=RAM_END => {
                let mirror_addr = addr & RAM_MASK;
                self.cpu_ram[mirror_addr as usize] = data;
            }

            PPU_START..=PPU_END => {
                let _ = addr & PPU_MASK;
                todo!("No PPU support yet")
            }

            ROM_START..=ROM_END => panic!("Attempt to write to ROM"),

            _ => {
                println!("Write to unmapped address: 0x{:04X}", addr);
            }
        }
    }
}
