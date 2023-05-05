use crate::cpu::AM;
use std::collections::HashMap;

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

lazy_static! {
    pub static ref CPU_OPS: Vec<OPS> = vec![
        OPS::new(0x69, "ADC", 2, 2, AM::Immediate),
        OPS::new(0x65, "ADC", 2, 3, AM::ZeroPage),
        OPS::new(0x75, "ADC", 2, 4, AM::ZeroPageX),
        OPS::new(0x6D, "ADC", 3, 4, AM::Absolute),
        OPS::new(0x7D, "ADC", 3, 4 /* +1 cross page */, AM::AbsoluteX),
        OPS::new(0x79, "ADC", 3, 4 /* +1 cross page */, AM::AbsoluteY),
        OPS::new(0x61, "ADC", 2, 6, AM::IndirectX),
        OPS::new(0x71, "ADC", 2, 5 /* +1 cross page */, AM::IndirectY),

        OPS::new(0x29, "AND", 2, 2, AM::Immediate),
        OPS::new(0x25, "AND", 2, 3, AM::ZeroPage),
        OPS::new(0x35, "AND", 2, 4, AM::ZeroPageX),
        OPS::new(0x2D, "AND", 3, 4, AM::Absolute),
        OPS::new(0x3D, "AND", 3, 4 /* +1 cross page */, AM::AbsoluteX),
        OPS::new(0x39, "AND", 3, 4 /* +1 cross page */, AM::AbsoluteY),
        OPS::new(0x21, "AND", 2, 6, AM::IndirectX),
        OPS::new(0x31, "AND", 2, 5 /* +1 cross page */, AM::IndirectY),


    ];

    pub static ref OPS_MAP: HashMap<u8, &'static OPS> = {
        let mut map = HashMap::new();
        for op in CPU_OPS.iter() {
            map.insert(op.code, op);
        }
        map
    };
}