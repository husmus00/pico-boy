use std::collections::HashMap;
use crate::registers::{
    Reg16,
    Reg16::*,
    Reg8,
    Reg8::*,
    Registers,
    Flag::*,
};

static f: u32 = 5;

pub struct System {
    pub(crate) memory: [u8; 0xFFFF + 1], // +1 because inclusive
    pub(crate) reg: Registers,
    pub(crate) funcs: HashMap<u8, Box<dyn Fn(&mut Self, u8)>>
}

// Represents the possible values of r8 for certain instructions
static R8: [Reg8; 8] = [B, C, D, E, H, L, H /* <- No. 6 (7th pos) is a placeholder */, A]; // 6 is [hl] and handled by the function

// Represents the possible values of r16 for certain instructions
static R16: [Reg16; 4] = [BC, DE, HL, SP];

// Represents the possible values of r16mem for certain instructions
static R16MEM: [Reg16; 4] = [BC, DE, HL, HL]; // HL is duplicated because HL+, HL-

// type OpcodeFunction = fn(&System, u8);
//
// static OPCODES: HashMap<u8, OpcodeFunction> = HashMap::from([
//     (0x00, System::nop),
// ]);

fn ld_imm16_sp(s: &mut System, opcode: u8) {
    let sp = s.reg.get16(SP);
    let address: usize = s.get_imm16() as usize;
    s.memory[address] = sp as u8; // TODO check how sp is implemented
}

fn test() {
    let mut h: HashMap<u8, Box<dyn Fn(&mut System, u8)>> = HashMap::default();
    let b = Box::new(ld_imm16_sp);
    h.insert(8, b);

    let opcode: u8 = 8;

    h.get(&opcode).unwrap()(&mut System::new(), 8);
}

impl System {
    pub fn new() -> Self {
        Self {
            memory: [0; 0xFFFF + 1],
            reg: Registers::default(),
            funcs: HashMap::default(),
        }
    }

    fn next_byte(&mut self) -> u8 {
        0 // TODO
    }

    pub fn execute_op(&mut self, opcode: u8) {

        // self.funcs.insert(1, Box::new(|| self.ld_imm16_sp()));

        //
        // Block 0 (first two bits are 00) https://gbdev.io/pandocs/CPU_Instruction_Set.html#block-0
        // Part 1 (16-bit operands)
        match opcode & 0b11001111 {
            0x00 => self.nop(opcode), // nop

            0x01 => self.ld_r16_imm16(opcode), // ld r16, imm16
            0x02 => self.ld_r16mem_a(opcode),  // ld [r16mem], a
            0x0A => self.ld_a_r16mem(opcode),  // ld a, [r16mem]
            0x08 => self.ld_imm16_sp(),        // ld [imm16], sp

            0x03 => self.inc_r16(opcode),      // inc r16
            0x0B => self.dec_r16(opcode),      // dec r16
            0x09 => self.add_hl_r16(opcode),   // add hl, r16

            _ => ()
        }
        // Part 2 (8-bit operands)
        match opcode & 0b11000111 {
            0x04 => self.inc_r8(opcode),     // inc r8
            0x05 => self.dec_r8(opcode),     // dec r8

            0x06 => self.ld_r8_imm8(opcode), // ld r8, imm8

            _ => ()
        }
        // Part 3 (no operands)
        match opcode {
            0x07 => self.rlca(),
            0x0F => self.rrca(),
            0x17 => self.rla(),
            0x1F => self.rra(),
            0x27 => self.daa(),
            0x2F => self.cpl(),
            0x37 => self.scf(),
            0x3F => self.ccf(),

            0x18 => self.jr_imm8(),

            0x10 => self.stop(),

            _ => ()
        }
        // Part 4 (conditional jr)
        match (opcode & 0b11100111) {
            0x20 => self.jr_cond_imm8(opcode), // jr cond, imm8

            _ => (),
        }

        //
        // Block 1 (first two bits are 01)
        // 8-bit register-to-register load instruction
        if ((opcode & 0b11000000) >> 6) == 0b01 {
            // Exception: trying to encode ld [hl], [hl] instead yields the halt instruction:
            if opcode == 0b01110110 {
                self.halt()
            }

            // Otherwise the operation is ld r8, r8
            self.ld_r8_r8(opcode);
        }

        //
        // Block 2 (first two bits are 10)
        // 8-bit arithmetic
        match opcode & 0b11111000 {

            0x80 => self.add_a_r8(opcode),
            0x88 => self.adc_a_r8(opcode),
            0x90 => self.sub_a_r8(opcode),
            0x98 => self.sbc_a_r8(opcode),
            0xA0 => self.and_a_r8(opcode),
            0xA8 => self.xor_a_r8(opcode),
            0xB0 => self.or_a_r8(opcode),
            0xB8 => self.cp_a_r8(opcode),

            _ => ()
        }

        //
        // Block 3 (first two bits are 11)
        // Part 1: Miscellaneous instructions with no opcode parameters
        match opcode {
            // Immediate 8-bit instructions
            0xC6 => self.add_a_imm8(opcode),
            0xCE => self.adc_a_imm8(opcode),
            0xD6 => self.sub_a_imm8(opcode),
            0xDE => self.sbc_a_imm8(opcode),
            0xE6 => self.and_a_imm8(opcode),
            0xEE => self.xor_a_imm8(opcode),
            0xF6 => self.or_a_imm8(opcode),
            0xFE => self.cp_a_imm8(opcode),

            // Jumps, Returns, Calls (non-conditional)
            0xC9 => self.ret(opcode),
            0xD9 => self.reti(opcode),
            0xC3 => self.jp_imm16(opcode),
            0xE9 => self.jp_hl(opcode),
            0xCD => self.call_imm16(opcode),

            // ld / ldh instructions
            0xE2 => self.ldh_c_a(opcode),
            0xE0 => self.ldh_imm8_a(opcode),
            0xEA => self.ld_imm16_a(opcode),
            0xF2 => self.ldh_a_c(opcode),
            0xF0 => self.ldh_a_imm8(opcode),
            0xFA => self.ld_a_imm16(opcode),

            // Instructions using sp
            0xE8 => self.add_sp_imm8(opcode),
            0xF8 => self.ld_hl_sp_plus_imm8(opcode),
            0xF9 => self.ld_sp_hl(opcode),

            // di and ei
            0xF3 => self.di(opcode),
            0xFB => self.ei(opcode),

            _ => (),
        }

        // Part 2: Miscellaneous instructions with parameters
        // Jumps/Calls
        match opcode & 0b11100111 {
            0xC0 => self.ret_cond(opcode),
            0xC2 => self.jp_cond_imm16(opcode),
            0xC4 => self.call_cond_imm16(opcode),
            _ => (),
        }
        // Reset
        match opcode & 0b11000111 {
            0xC7 => self.rst_tgt3(opcode),
            _ => (),
        }
        // Stack
        match opcode & 0b11001111 {
            0xC1 => self.pop_r16stk(opcode),
            0xC5 => self.push_r16stk(opcode),
            _ => (),
        }

        // $CB prefix instructions
        if opcode == 0xCB {
            // The next byte needs to be retrieved
            let next_byte = self.next_byte();

            // First set
            match next_byte & 0b11111000 {
                0x00 => self.rlc_r8(opcode),
                0x08 => self.rrc_r8(opcode),
                0x10 => self.rl_r8(opcode),
                0x18 => self.rr_r8(opcode),
                0x20 => self.sla_r8(opcode),
                0x28 => self.sra_r8(opcode),
                0x30 => self.swap_r8(opcode),
                0x38 => self.srl_r8(opcode),
                _ => (),
            }
            // Second set
            match next_byte & 0b11000000 {
                0x40 => self.bit_b3_r8(opcode),
                0x80 => self.res_b3_r8(opcode),
                0xC0 => self.set_b3_r8(opcode),
                _ => (),
            }
        }

        // If no opcode matches, it must be invalid
        // self.invalid_opcode(opcode);
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ld_a_r16mem() {
        let mut s = System::new();

        s.memory[0] = 20;
        s.execute_op(0b00111010); // ld a, [hl-]
        // Test that reg a was loaded with 20 and that hl was decremented
        assert_eq!(s.reg.get8(A), 20);
        assert_eq!(s.reg.get16(HL), 65535);

        s.memory[65535] = 30; // Because hl was decremented and now pointing to the end of memory
        s.execute_op(0b00101010); // ld a, [hl+]
        // Test that hl was incremented back to 0
        assert_eq!(s.reg.get8(A), 30);
        assert_eq!(s.reg.get16(HL), 0);
    }

    #[test]
    fn test_inc_r16() {
        let mut s = System::new();
        s.reg.set16(BC, 0);
        s.execute_op(0b00000011);
        assert_eq!(s.reg.get16(BC), 1);
    }

    #[test]
    fn test_dec_r16() {
        let mut s = System::new();
        s.reg.set16(BC, 0);
        s.execute_op(0b00001011);
        assert_eq!(s.reg.get16(BC), 65535);
    }

    #[test]
    fn test_add_hl_r16() {
        let mut s = System::new();
        s.reg.set16(BC, 5);
        s.reg.set16(HL, 65535);
        s.execute_op(0b00001001);
        assert_eq!(s.reg.get16(HL), 4);
    }

    #[test]
    fn test_inc_r8() {
        let mut s = System::new();
        s.reg.set16(BC, 0);
        s.execute_op(0b00001100); // inc c
        assert_eq!(s.reg.get16(BC), 1);
    }

    #[test]
    fn test_rlca() {
        let mut s = System::new();
        s.reg.set8(A, 0b11110000);
        s.execute_op(0b00000111); // rlca
        // Ensure that reg a was rotated left and carry was set to 1
        assert_eq!(s.reg.get8(A), 0b11100001);
        assert_eq!(s.reg.get_flag(Carry), 1);
    }

    #[test]
    fn test_rrca() {
        let mut s = System::new();
        s.reg.set8(A, 0b11110000);
        s.execute_op(0b00001111); // rrca
        // Ensure that reg a was rotated right and carry was set to 0
        assert_eq!(s.reg.get8(A), 0b01111000);
        assert_eq!(s.reg.get_flag(Carry), 0);
    }

    #[test]
    fn test_ld_r8_r8() {
        let mut s = System::new();
        s.reg.set8(A, 0b11110000);
        s.execute_op(0b01000111); // ld b, a
        assert_eq!(s.reg.get8(B), 0b11110000);
    }
}