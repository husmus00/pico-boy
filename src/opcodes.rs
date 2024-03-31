use std::collections::HashMap;
use crate::system::System;
use crate::registers::{Reg16, Reg8};
use crate::registers::Flag::{Carry, HalfCarry, Sub, Zero};
use crate::registers::Reg16::{HL, SP};
use crate::registers::Reg8::A;

impl System {
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

        let r8: Reg8 = crate::system::R8[reg_no];

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

        let r8: Reg8 = crate::system::R8[reg_no];

        return r8
    }

    /// Gets the 16-bit (r16) register specified in the opcode
    fn get_r16(&self, opcode: u8) -> Reg16 {
        let reg_no: usize = ((opcode & 0x30) >> 4) as usize;
        crate::system::R16[reg_no]
    }

    /// Gets the 16-bit memory address (r16mem) from the register specified in the opcode
    fn get_r16mem_address(&mut self, opcode: u8) -> usize {
        let reg_no: usize = ((opcode & 0b00110000) >> 4) as usize; // 2 bits

        let old_val = self.reg.get16(crate::system::R16MEM[reg_no]);

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