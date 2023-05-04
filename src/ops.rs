use crate::cpu::AM;

pub struct OPS {
    pub code: u8,
    pub name: & 'static str,
    pub bytes: u8,
    pub cycles: u8,
    pub mode: AM,
}

impl OPS {
    pub fn new(code: u8, name: &'static str, bytes: u8, cycles: u8, mode: AM) -> Self {
        OPS {
            code,
            name,
            bytes,
            cycles,
            mode,
        }
    }
}