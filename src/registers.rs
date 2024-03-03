use std::num::Wrapping;

#[derive(Clone, Copy)]
pub enum Reg8 {
    A,
    B,
    C,
    D,
    E,
    H,
    L,
}

#[derive(Clone, Copy)]
pub enum Reg16 {
    AF,
    BC,
    DE,
    HL,
    SP,
    PC,
}

#[derive(Clone, Copy)]
pub enum Flag {
    Zero,
    Sub,
    HalfCarry,
    Carry,
}
#[derive(Default)]
pub struct Registers {
    af: u16,
    de: u16,
    bc: u16,
    hl: u16,
    sp: u16,
    pc: u16,
}

impl Registers {
    /// Get the value of an 8 bit (half-width) register
    pub fn get8(&self, reg: Reg8) -> u8 {
        match reg {
            // Half-width registers
            Reg8::A => (self.af >> 8) as u8,
            Reg8::B => (self.bc >> 8) as u8,
            Reg8::C => self.bc as u8,
            Reg8::D => (self.de >> 8) as u8,
            Reg8::E => self.de as u8,
            Reg8::H => (self.hl >> 8) as u8,
            Reg8::L => self.hl as u8
        }
    }

    /// Get the value of a 16 bit (full width) register
    pub fn get16(&self, reg: Reg16) -> u16 {
        match reg {
            // Full-width registers
            Reg16::AF => self.af,
            Reg16::BC => self.bc,
            Reg16::DE => self.de,
            Reg16::HL => self.hl,
            Reg16::SP => self.sp,
            Reg16::PC => self.pc
        }
    }

    /// Set the value of an 8 bit (half-width) register
    pub fn set8(&mut self, reg: Reg8, val: u8) {
        match reg {
            // Half-width registers
            Reg8::A => self.af = (self.af & 0x00FF) | ((val as u16) << 8),
            Reg8::B => self.bc = (self.bc & 0x00FF) | ((val as u16) << 8),
            Reg8::C => self.bc = (self.bc & 0xFF00) | (val as u16),
            Reg8::D => self.de = (self.de & 0x00FF) | ((val as u16) << 8),
            Reg8::E => self.de = (self.de & 0xFF00) | (val as u16),
            Reg8::H => self.hl = (self.hl & 0x00FF) | ((val as u16) << 8),
            Reg8::L => self.hl = (self.hl & 0xFF00) | (val as u16)
        }
    }

    /// Set the value of a 16 bit (full width) register
    pub fn set16(&mut self, reg: Reg16, val: u16) {
        match reg {
            // Full-width registers
            Reg16::AF => self.af = val,
            Reg16::BC => self.bc = val,
            Reg16::DE => self.de = val,
            Reg16::HL => self.hl = val,
            Reg16::SP => self.sp = val,
            Reg16::PC => self.pc = val
        };
    }

    /// Add the value of 'operand' to register 'reg'. We use 'Wrapping' to allow for overflow
    pub fn add8(&mut self, reg: Reg8, operand: i8) {
        let current_val = self.get8(reg);
        let sum = (Wrapping(current_val) + Wrapping(operand as u8)).0;
        self.set8(reg, sum);
    }

    /// Add the value of 'operand' to register 'reg'. We use 'Wrapping' to allow for overflow
    pub fn add16(&mut self, reg: Reg16, operand: i16) {
        let current_val = self.get16(reg);
        let sum = (Wrapping(current_val) + Wrapping(operand as u16)).0;
        self.set16(reg, sum);
    }

    /// Get the boolean value of a flag
    pub fn get_flag(&self, flag: Flag) -> u8 {
        // Registers are set in the upper nibble of F in AF
        let result = match flag {
            Flag::Zero => (self.af & 0x0080) >> 7,
            Flag::Sub => (self.af & 0x0040) >> 6,
            Flag::HalfCarry => (self.af & 0x0020) >> 5,
            Flag::Carry => (self.af & 0x0010) >> 4
        } as u8;

        return result;
    }

    /// Set the boolean value of a flag
    pub fn set_flag(&mut self, flag: Flag, val: u8) {
        if val > 0 {
            // Then we need to set the flag
            match flag {
                Flag::Zero => self.af |= 1 << 7,
                Flag::Sub => self.af |= 1 << 6,
                Flag::HalfCarry => self.af |= 1 << 5,
                Flag::Carry=> self.af |= 1 << 4
            };

            return;
        }

        // Else clear the flag
        match flag {
            Flag::Zero => self.af &= !(1 << 7),
            Flag::Sub => self.af &= !(1 << 6),
            Flag::HalfCarry => self.af &= !(1 << 5),
            Flag::Carry=> self.af &= !(1 << 4)
        };
    }

}