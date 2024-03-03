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

    ///
    /// Instructions: // TODO handle flags
    ///

    fn nop(&self, opcode: u8) {

    }

    /// ld r16, imm16
    fn ld_r16_imm16(&mut self, opcode: u8) {
        let value = self.get_imm16();
        let reg = self.get_r16(opcode);
        self.reg.set16(reg, value);
    }

    /// ld [r16mem], a
    fn ld_r16mem_a(&mut self, opcode: u8) {
        let reg_a_val = self.reg.get8(A);
        let address = self.get_r16mem_address(opcode);
        self.memory[address] = reg_a_val;
    }

    /// ld a, [r16mem]
    fn ld_a_r16mem(&mut self, opcode: u8) {
        let address = self.get_r16mem_address(opcode);
        let value = self.memory[address];
        self.reg.set8(A, value);
    }

    /// ld [imm16], sp
    fn ld_imm16_sp(&mut self) {
        let sp = self.reg.get16(SP);
        let address: usize = self.get_imm16() as usize;
        self.memory[address] = sp as u8; // TODO check how sp is implemented
    }

    /// inc r16
    fn inc_r16(&mut self, opcode: u8) {
        let reg = self.get_r16(opcode);
        self.reg.add16(reg, 1)
    }

    /// dec r16
    fn dec_r16(&mut self, opcode: u8) {
        let reg = self.get_r16(opcode);
        self.reg.add16(reg, -1)
    }

    /// add hl, r16
    fn add_hl_r16(&mut self, opcode: u8) {
        let reg = self.get_r16(opcode);
        let reg_val = self.reg.get16(reg);
        self.reg.add16(HL, reg_val as i16)
    }

    /// inc r8
    fn inc_r8(&mut self, opcode: u8) {
        let reg = self.get_upper_r8(opcode);
        self.reg.add8(reg, 1)
    }

    /// dec r8
    fn dec_r8(&mut self, opcode: u8) {
        let reg = self.get_upper_r8(opcode);
        self.reg.add8(reg, -1)
    }

    /// ld r8, imm8
    fn ld_r8_imm8(&mut self, opcode: u8) {
        let reg = self.get_upper_r8(opcode);
        let imm8 = self.get_imm8();
        self.reg.set8(reg, imm8);
    }

    /// rlca (Rotate register A left) https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#RLCA
    fn rlca(&mut self) {
        let a = self.reg.get8(A);
        let new_val = (a << 1) | ((a & 0b10000000) >> 7);
        self.reg.set8(A, new_val);

        let carry = (a & 0b10000000) >> 7;
        self.set_flags(0, 0, 0, carry);
    }

    /// rrca (Rotate register A right.) https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#RRCA
    fn rrca(&mut self) {
        let a = self.reg.get8(A);
        let new_val = ((a & 0b00000001) << 7) | (a >> 1);
        self.reg.set8(A, new_val);

        let carry = a & 0b00000001;
        self.set_flags(0, 0, 0, carry);
    }

    /// rla (Rotate register A left, through the carry flag) https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#RLA
    fn rla(&mut self) {
        let a = self.reg.get8(A);
        let new_val = (a << 1) | (self.reg.get_flag(Carry));
        self.reg.set8(A, new_val);

        let carry = a & 0b10000000;
        self.set_flags(0, 0, 0, carry);
    }

    /// rra (Rotate register A right, through the carry flag) https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#RRA
    fn rra(&mut self) {
        let a = self.reg.get8(A);
        let new_val = (self.reg.get_flag(Carry) << 7) | (a >> 1);
        self.reg.set8(A, new_val);

        let carry = a & 0b00000001;
        self.set_flags(0, 0, 0, carry);
    }

    /// daa ()
    /// The Subtraction flag and Half Carry flag are only used here
    fn daa(&mut self) {
        // TODO
    }

    /// cpl (ComPLement accumulator A = ~A)
    fn cpl(&mut self) {
        let a = self.reg.get8(A);
        self.reg.set8(A, !a);

        self.set_flags(
            self.reg.get_flag(Zero),
            1,
            1,
            self.reg.get_flag(Carry),
        );
    }

    /// scf (Set Carry Flag)
    fn scf(&mut self) {
        self.set_flags(
            self.reg.get_flag(Zero),
            0,
            0,
            1,
        );
    }

    /// ccf (Complement Carry Flag)
    fn ccf(&mut self) {
        self.set_flags(
            self.reg.get_flag(Zero),
            0,
            0,
            !self.reg.get_flag(Carry),
        );
    }

    /// jr imm8 https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#JR_n16
    fn jr_imm8(&mut self) {
        // TODO
    }

    /// stop (Enter CPU very low power mode)
    fn stop(&mut self) {
        // TODO
    }

    /// jr cond, imm8
    fn jr_cond_imm8(&mut self, opcode: u8) {
        // TODO
    }

    /// halt https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#HALT
    fn halt(&mut self) {
        // TODO
    }

    /// ld r8, r8 (Load/copy value in register on the right into register on the left)
    fn ld_r8_r8(&mut self, opcode: u8) {
        let right_reg = self.get_lower_r8(opcode);
        let left_reg = self.get_upper_r8(opcode);

        let right_reg_value = self.reg.get8(right_reg);
        self.reg.set8(left_reg, right_reg_value);
    }

    /// add a, r8 https://rgbds.gbdev.io/docs/v0.7.0/gbz80.7#ADD_A,r8
    fn add_a_r8(&mut self, opcode: u8) {
        let reg = self.get_lower_r8(opcode);
        let reg_val = self.reg.get8(reg);
        self.reg.add8(A, reg_val as i8);

        let new_val = self.reg.get8(A);
        // Determine values of changed flags
        let zero: u8 = match new_val {
            0 => 0,
            _ => 1,
        };

        let carry = self.check_carry(reg_val, new_val);
        let half_carry = self.check_half_carry_r8(0, 0); // TODO half

        self.set_flags(
            zero,
            0,
            half_carry,
            carry,
        );
    }

    /// adc a, r8
    fn adc_a_r8(&mut self, opcode: u8) {

    }

    /// sub a, r8
    fn sub_a_r8(&mut self, opcode: u8) {

    }

    /// sbc a, r8
    fn sbc_a_r8(&mut self, opcode: u8) {

    }

    /// and a, r8
    fn and_a_r8(&mut self, opcode: u8) {

    }

    /// xor a, r8
    fn xor_a_r8(&mut self, opcode: u8) {

    }

    /// or a, r8
    fn or_a_r8(&mut self, opcode: u8) {

    }

    /// cp a, r8
    fn cp_a_r8(&mut self, opcode: u8) {

    }

    /// add a, imm8
    fn add_a_imm8(&mut self, opcode: u8) {

    }

    /// adc a, imm8
    fn adc_a_imm8(&mut self, opcode: u8) {

    }

    /// sub a, imm8
    fn sub_a_imm8(&mut self, opcode: u8) {

    }

    /// sbc a, imm8
    fn sbc_a_imm8(&mut self, opcode: u8) {

    }

    /// and a, imm8
    fn and_a_imm8(&mut self, opcode: u8) {

    }

    /// xor a, imm8
    fn xor_a_imm8(&mut self, opcode: u8) {

    }

    /// or a, imm8
    fn or_a_imm8(&mut self, opcode: u8) {

    }

    /// cp a, imm8
    fn cp_a_imm8(&mut self, opcode: u8) {

    }

    /// ret
    fn ret(&mut self, opcode: u8) {

    }

    /// reti
    fn reti(&mut self, opcode: u8) {

    }

    /// jp imm16
    fn jp_imm16(&mut self, opcode: u8) {

    }

    /// jp hl
    fn jp_hl(&mut self, opcode: u8) {

    }

    /// call imm16
    fn call_imm16(&mut self, opcode: u8) {

    }

    /// ldh c, a
    fn ldh_c_a(&mut self, opcode: u8) {

    }

    /// ldh imm8, a
    fn ldh_imm8_a(&mut self, opcode: u8) {

    }

    /// ld imm16, a
    fn ld_imm16_a(&mut self, opcode: u8) {

    }

    /// ldh a, c
    fn ldh_a_c(&mut self, opcode: u8) {

    }

    /// ldh a, imm8
    fn ldh_a_imm8(&mut self, opcode: u8) {

    }

    /// ld a, imm16
    fn ld_a_imm16(&mut self, opcode: u8) {

    }

    /// add sp, imm8
    fn add_sp_imm8(&mut self, opcode: u8) {

    }

    /// ld hl, sp+imm8
    fn ld_hl_sp_plus_imm8(&mut self, opcode: u8) {

    }

    /// ld sp, hl
    fn ld_sp_hl(&mut self, opcode: u8) {

    }

    /// di
    fn di(&mut self, opcode: u8) {

    }

    /// ei
    fn ei(&mut self, opcode: u8) {

    }

    /// ret cond
    fn ret_cond(&mut self, opcode: u8) {

    }

    /// jp cond, imm16
    fn jp_cond_imm16(&mut self, opcode: u8) {

    }

    /// call cond, imm16
    fn call_cond_imm16(&mut self, opcode: u8) {

    }

    /// rst tgt3
    fn rst_tgt3(&mut self, opcode: u8) {

    }

    /// pop r16stk
    fn pop_r16stk(&mut self, opcode: u8) {

    }

    /// push r16stk
    fn push_r16stk(&mut self, opcode: u8) {

    }

    // $CB Prefix instructions

    /// rlc r8
    fn rlc_r8(&mut self, opcode: u8) {

    }

    /// rrc r8
    fn rrc_r8(&mut self, opcode: u8) {

    }

    /// rl r8
    fn rl_r8(&mut self, opcode: u8) {

    }

    /// rr_r8
    fn rr_r8(&mut self, opcode: u8) {

    }

    /// sla r8
    fn sla_r8(&mut self, opcode: u8) {

    }

    /// sra r8
    fn sra_r8(&mut self, opcode: u8) {

    }

    /// swap r8
    fn swap_r8(&mut self, opcode: u8) {

    }

    /// srl r8
    fn srl_r8(&mut self, opcode: u8) {

    }

    /// res b3, r8
    fn res_b3_r8(&mut self, opcode: u8) {

    }

    /// set b3, r8
    fn set_b3_r8(&mut self, opcode: u8) {

    }

    /// bit b3, r8
    fn bit_b3_r8(&mut self, opcode: u8) {

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