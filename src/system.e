use std::collections::HashMap;

use crate::registers::{
    Reg16,
    Reg16::*,
    Reg8,
    Reg8::*,
    Registers,
    Flag::*,
};

// Represents the possible values of r8 for certain instructions
static R8: [Reg8; 8] = [B, C, D, E, H, L, H /* <- No. 6 (7th pos) is a placeholder */, A]; // 6 is [hl] and handled by the function

// Represents the possible values of r16 for certain instructions
static R16: [Reg16; 4] = [BC, DE, HL, SP];

// Represents the possible values of r16mem for certain instructions
static R16MEM: [Reg16; 4] = [BC, DE, HL, HL]; // HL is duplicated because HL+, HL-

// fn test() {
//     let mut h: HashMap<u8, Box<dyn Fn(&mut System, u8)>> = HashMap::default();
//     let b = Box::new(ld_imm16_sp);
//     h.insert(8, b);
//
//     let opcode: u8 = 8;
//
//     h.get(&opcode).unwrap()(&mut System::new(), 8);
// }

// Represents one of the opcode functions
type OpcodeFunction = Box<dyn Fn(&mut System, u8)>;

struct OPS;

// impl OPS {
//     fn get(t: u8) -> Option<&'static OpcodeFunction> {
//
//         let x = map.get(&t).unwrap();
//         return Some(x.clone().to_owned());
//     }
// }

pub struct System {
    pub(crate) memory: [u8; 0xFFFF + 1], // +1 because inclusive
    pub(crate) reg: Registers,
    // pub(crate) ops: HashMap<u8, OpcodeFunction>
}

/// Returns a boxed version to avoid constantly typing Box::new() for the
/// function pointer trait objects
fn op<F>(opcode: u8, func: F) -> (u8, OpcodeFunction)
where F: Fn(&mut System, u8) + 'static
{
    (opcode, Box::new(func))
}

// static OPS: HashMap<u8, OpcodeFunction> =

impl System {
    pub fn new() -> Self {

        // Register all the functions
        // let mut op_table =

        let mut x = Self {
            memory: [0; 0xFFFF + 1],
            reg: Registers::default(),
            // ops: op_table,
        };

        return x;
    }

    pub fn execute_op(&mut self, opcode: u8) {

        // Explanation: After setting the variable bits of any opcode to 0, we will always end up with
        // a unique number to index into this "function table".
        let ops = HashMap::from([
            op(0x00, nop),

            op(0x01, ld_r16_imm16),
            op(0x0A, ld_a_r16mem),
            op(0x03, inc_r16),
            op(0x0B, dec_r16),
            op(0x09, add_hl_r16),

            op(0x04, inc_r8),

            op(0x07, rlca),
            op(0x0F, rrca),

        ]);

        //
        // Block 0 (first two bits are 00) https://gbdev.io/pandocs/CPU_Instruction_Set.html#block-0
        // Part 1 (16-bit operands)
        let test = opcode & 0b11001111;
        if let Some(op) = ops.get(&test) {
            op(self, opcode);
            return;
        }

        // match opcode & 0b11001111 {
        //     0x00 => self.nop(opcode), // nop
        //
        //     0x01 => self.ld_r16_imm16(opcode), // ld r16, imm16
        //     0x02 => self.ld_r16mem_a(opcode),  // ld [r16mem], a
        //     0x0A => self.ld_a_r16mem(opcode),  // ld a, [r16mem]
        //     0x08 => self.ld_imm16_sp(),        // ld [imm16], sp
        //
        //     0x03 => self.inc_r16(opcode),      // inc r16
        //     0x0B => self.dec_r16(opcode),      // dec r16
        //     0x09 => self.add_hl_r16(opcode),   // add hl, r16
        //
        //     _ => ()
        // }

        // Part 2 (8-bit operands)
        let test = opcode & 0b11000111;
        if let Some(op) = ops.get(&test) {
            op(self, opcode);
            return;
        }

        // match opcode & 0b11000111 {
        //     0x04 => self.inc_r8(opcode),     // inc r8
        //     0x05 => self.dec_r8(opcode),     // dec r8
        //
        //     0x06 => self.ld_r8_imm8(opcode), // ld r8, imm8
        //
        //     _ => ()
        // }

        // Part 3 (no operands)
        if let Some(op) = ops.get(&test) {
            op(self, opcode);
            return;
        }

        // match opcode {
        //     0x07 => self.rlca(),
        //     0x0F => self.rrca(),
        //     0x17 => self.rla(),
        //     0x1F => self.rra(),
        //     0x27 => self.daa(),
        //     0x2F => self.cpl(),
        //     0x37 => self.scf(),
        //     0x3F => self.ccf(),
        //
        //     0x18 => self.jr_imm8(),
        //
        //     0x10 => self.stop(),
        //
        //     _ => ()
        // }
        // Part 4 (conditional jr)

        match (opcode & 0b11100111) {
            0x20 => jr_cond_imm8(self, opcode), // jr cond, imm8

            _ => (),
        }

        //
        // Block 1 (first two bits are 01)
        // 8-bit register-to-register load instruction
        if ((opcode & 0b11000000) >> 6) == 0b01 {
            // Exception: trying to encode ld [hl], [hl] instead yields the halt instruction:
            if opcode == 0b01110110 {
                halt(self, opcode)
            }

            // Otherwise the operation is ld r8, r8
            ld_r8_r8(self, opcode);
        }

        //
        // Block 2 (first two bits are 10)
        // 8-bit arithmetic
        let test = opcode & 0b11111000;
        if let Some(op) = ops.get(&test) {
            op(self, opcode);
            return;
        }

        // match opcode & 0b11111000 {
        //
        //     0x80 => self.add_a_r8(opcode),
        //     0x88 => self.adc_a_r8(opcode),
        //     0x90 => self.sub_a_r8(opcode),
        //     0x98 => self.sbc_a_r8(opcode),
        //     0xA0 => self.and_a_r8(opcode),
        //     0xA8 => self.xor_a_r8(opcode),
        //     0xB0 => self.or_a_r8(opcode),
        //     0xB8 => self.cp_a_r8(opcode),
        //
        //     _ => ()
        // }

        // If no opcode matches, it must be invalid
        // self.invalid_opcode(opcode);
    }

    ///
    /// Helper functions
    ///

    fn invalid_opcode(&self, opcode: u8) {
        panic!("Invalid opcode: \"{:X}\"", opcode)
    }

    // TODO
    /// Get the next byte (opcode operand) and further increment the program counter
    fn get_imm8(&self) -> u8 {
        0
    }

    // TODO
    /// Get the next 2 bytes (opcode operand) and further increment the program counter
    fn get_imm16(&self) -> u16 {
        0
    }

    /// Gets the 8-bit (r8) register specified in bits
    /// *5, 4, 3* of the opcode, required by certain opcodes
    fn get_upper_r8(&self, opcode: u8) -> Reg8 {
        let reg_no : usize = ((opcode & 0b00111000) >> 3) as usize;

        // Handle [hl]
        if reg_no == 6 {
            // TODO
        }

        let r8: Reg8 = R8[reg_no];

        return r8
    }

    /// Gets the 8-bit (r8) register specified in bits
    /// *2, 1, 0* of the opcode, required for certain functions
    fn get_lower_r8(&self, opcode: u8) -> Reg8 {
        let reg_no : usize = (opcode & 0b00000111) as usize;

        // Handle [hl]
        if reg_no == 6 {
            // TODO
        }

        let r8: Reg8 = R8[reg_no];

        return r8
    }

    /// Gets the 16-bit (r16) register specified in the opcode
    fn get_r16(&self, opcode: u8) -> Reg16 {
        let reg_no: usize = ((opcode & 0x30) >> 4) as usize;
        R16[reg_no]
    }

    /// Gets the 16-bit memory address (r16mem) from the register specified in the opcode
    fn get_r16mem_address(&mut self, opcode: u8) -> usize {
        let reg_no: usize = ((opcode & 0b00110000) >> 4) as usize; // 2 bits

        let old_val = self.reg.get16(R16MEM[reg_no]);

        // numbers 2 and 3 correspond to HL+ and HL-, respectively
        // HL is incremented or decremented after the operation
        // (so we return the original HL value)
        if reg_no == 2 {
            self.reg.add16(HL, 1);
        }
        if reg_no == 3 {
            self.reg.add16(HL, -1);
        }

        return old_val as usize;
    }

    /// Set the Zero, Subtraction (Negative), Half carry, and Carry flags
    /// If a flag is not changed, the function should be supplied with the existing value
    fn set_flags(&mut self, z: u8, n: u8, h: u8, c: u8) {
        self.reg.set_flag(Zero, z);
        self.reg.set_flag(Sub, n);
        self.reg.set_flag(HalfCarry, h);
        self.reg.set_flag(Carry, c);
    }

    /// Check if a carry has occurred and return the should-be value of the Carry flag
    fn check_carry<T: PartialOrd>(&self, old_val: T, new_val: T) -> u8 {
        match old_val > new_val {
            true => 1,
            false => 0,
        }
    }

    /// Check if a half carry has occurred and return the should-be value of the HalfCarry flag
    fn check_half_carry_r8(&self, old_value: u8, new_value: u8) -> u8 {
        0
    }
}

///
/// Instructions: // TODO handle flags
///

fn nop(s: &mut System, opcode: u8) {

}

/// ld r16, imm16
fn ld_r16_imm16(s: &mut System, opcode: u8) {
    let value = s.get_imm16();
    let reg = s.get_r16(opcode);
    s.reg.set16(reg, value);
}

/// ld [r16mem], a
fn ld_r16mem_a(s: &mut System, opcode: u8) {
    let reg_a_val = s.reg.get8(A);
    let address = s.get_r16mem_address(opcode);
    s.memory[address] = reg_a_val;
}

/// ld a, [r16mem]
fn ld_a_r16mem(s: &mut System, opcode: u8) {
    let address = s.get_r16mem_address(opcode);
    let value = s.memory[address];
    s.reg.set8(A, value);
}

/// ld [imm16], sp
fn ld_imm16_sp(s: &mut System) {
    let sp = s.reg.get16(SP);
    let address: usize = s.get_imm16() as usize;
    s.memory[address] = sp as u8; // TODO check how sp is implemented
}

/// inc r16
fn inc_r16(s: &mut System, opcode: u8) {
    let reg = s.get_r16(opcode);
    s.reg.add16(reg, 1);
}

/// dec r16
fn dec_r16(s: &mut System, opcode: u8) {
    let reg = s.get_r16(opcode);
    s.reg.add16(reg, -1)
}

/// add hl, r16
fn add_hl_r16(s: &mut System, opcode: u8) {
    let reg = s.get_r16(opcode);
    let reg_val = s.reg.get16(reg);
    s.reg.add16(HL, reg_val as i16)
}

/// inc r8
fn inc_r8(s: &mut System, opcode: u8) {
    let reg = s.get_upper_r8(opcode);
    s.reg.add8(reg, 1)
}

/// dec r8
fn dec_r8(s: &mut System, opcode: u8) {
    let reg = s.get_upper_r8(opcode);
    s.reg.add8(reg, -1)
}

/// ld r8, imm8
fn ld_r8_imm8(s: &mut System, opcode: u8) {
    let reg = s.get_upper_r8(opcode);
    let imm8 = s.get_imm8();
    s.reg.set8(reg, imm8);
}

/// rlca (Rotate register A left) https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#RLCA
fn rlca(s: &mut System, opcode: u8) {
    println!("Running RLCA");
    let a = s.reg.get8(A);
    let new_val = (a << 1) | ((a & 0b10000000) >> 7);
    s.reg.set8(A, new_val);

    let carry = (a & 0b10000000) >> 7;
    s.set_flags(0, 0, 0, carry);
}

/// rrca (Rotate register A right.) https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#RRCA
fn rrca(s: &mut System, opcode: u8) {
    let a = s.reg.get8(A);
    let new_val = ((a & 0b00000001) << 7) | (a >> 1);
    s.reg.set8(A, new_val);

    let carry = a & 0b00000001;
    s.set_flags(0, 0, 0, carry);
}

/// rla (Rotate register A left, through the carry flag) https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#RLA
fn rla(s: &mut System, opcode: u8) {
    let a = s.reg.get8(A);
    let new_val = (a << 1) | (s.reg.get_flag(Carry));
    s.reg.set8(A, new_val);

    let carry = a & 0b10000000;
    s.set_flags(0, 0, 0, carry);
}

/// rra (Rotate register A right, through the carry flag) https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#RRA
fn rra(s: &mut System, opcode: u8) {
    let a = s.reg.get8(A);
    let new_val = (s.reg.get_flag(Carry) << 7) | (a >> 1);
    s.reg.set8(A, new_val);

    let carry = a & 0b00000001;
    s.set_flags(0, 0, 0, carry);
}

/// daa ()
/// The Subtraction flag and Half Carry flag are only used here
fn daa(s: &mut System, opcode: u8) {
    // TODO
}

/// cpl (ComPLement accumulator A = ~A)
fn cpl(s: &mut System, opcode: u8) {
    let a = s.reg.get8(A);
    s.reg.set8(A, !a);

    s.set_flags(
        s.reg.get_flag(Zero),
        1,
        1,
        s.reg.get_flag(Carry),
    );
}

/// scf (Set Carry Flag)
fn scf(s: &mut System, opcode: u8) {
    s.set_flags(
        s.reg.get_flag(Zero),
        0,
        0,
        1,
    );
}

/// ccf (Complement Carry Flag)
fn ccf(s: &mut System, opcode: u8) {
    s.set_flags(
        s.reg.get_flag(Zero),
        0,
        0,
        !s.reg.get_flag(Carry),
    );
}

/// jr imm8 https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#JR_n16
fn jr_imm8(s: &mut System) {
    // TODO
}

/// stop (Enter CPU very low power mode)
fn stop(s: &mut System) {
    // TODO
}

/// jr cond, imm8
fn jr_cond_imm8(s: &mut System, opcode: u8) {
    // TODO
}

/// halt https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#HALT
fn halt(s: &mut System, opcode: u8) {
    // TODO
}

/// ld r8, r8 (Load/copy value in register on the right into register on the left)
fn ld_r8_r8(s: &mut System, opcode: u8) {
    let right_reg = s.get_lower_r8(opcode);
    let left_reg = s.get_upper_r8(opcode);

    let right_reg_value = s.reg.get8(right_reg);
    s.reg.set8(left_reg, right_reg_value);
}

/// add a, r8 https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#ADD_A,r8
fn add_a_r8(s: &mut System, opcode: u8) {
    let reg = s.get_lower_r8(opcode);
    let reg_val = s.reg.get8(reg);
    s.reg.add8(A, reg_val as i8);

    let new_val = s.reg.get8(A);
    // Determine values of changed flags
    let zero: u8 = match new_val {
        0 => 0,
        _ => 1,
    };

    let carry = s.check_carry(reg_val, new_val);
    let half_carry = s.check_half_carry_r8(0, 0); // TODO half

    s.set_flags(
        zero,
        0,
        half_carry,
        carry,
    );
}

/// adc a, r8
fn adc_a_r8(s: &mut System, opcode: u8) {

}

/// sub a, r8
fn sub_a_r8(s: &mut System, opcode: u8) {

}

/// sbc a, r8
fn sbc_a_r8(s: &mut System, opcode: u8) {

}

/// and a, r8
fn and_a_r8(s: &mut System, opcode: u8) {

}

/// xor a, r8
fn xor_a_r8(s: &mut System, opcode: u8) {

}

/// or a, r8
fn or_a_r8(s: &mut System, opcode: u8) {

}

/// cp a, r8
fn cp_a_r8(s: &mut System, opcode: u8) {

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