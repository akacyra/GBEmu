#ifndef GUARD_CPU_H
#define GUARD_CPU_H

#include <stdint.h>

#define BIT(X, N) ((X) & (1 << (N)))

// 8bit registers
// Lower 8 bits of 16bit registers stored first due to endianess. 
const uint8_t REG_A = 1;
const uint8_t REG_F = 0;

const uint8_t REG_B = 3;
const uint8_t REG_C = 2;

const uint8_t REG_D = 5;
const uint8_t REG_E = 4;

const uint8_t REG_H = 7;
const uint8_t REG_L = 6;

// 16bit registers
const uint8_t REG_AF = 0;
const uint8_t REG_BC = 1;
const uint8_t REG_DE = 2;
const uint8_t REG_HL = 3;

// Zero Flag
const uint8_t FLAG_Z = 0x80;
// Subtract Flag
const uint8_t FLAG_N = 0x40;
// Half Carry Flag
const uint8_t FLAG_H = 0x20;
// Carry Flag 
const uint8_t FLAG_C = 0x10;

const uint8_t NUM_REG8 = 8;
const uint8_t NUM_REG16 = 4;

const uint16_t MEM_SIZE = 0xffff;


class CPU {
    public:
        typedef uint8_t Reg8;
        typedef uint16_t Reg16;

        CPU();

        void print_registers() const;
        void print_register(uint8_t reg_id) const;
        void print_mem(uint16_t addr) const;
        void print_mem(uint16_t begin, uint16_t end) const;

    private:
        // Program Counter
        Reg16 pc; 
        // Stack Pointer 
        Reg16 sp; 
        // General purpose 8bit registers
        Reg8 reg8[NUM_REG8];

        // 16bit registers are just pairs of 8bit registers.
        // Cast as Reg16* to get { AF, BC, DE, HL }.
        Reg16 &reg16(uint8_t reg_id) { return ((Reg16 *)reg8)[reg_id]; };

        uint8_t mem[MEM_SIZE];

        bool interrupts;

    //
    // Instruction Functions
    //

        // Load val (reg, imm, or mem) into dest(reg or mem).
        void ld8(uint8_t &dest, uint8_t val) { dest = val; };
        void ld16(uint16_t &dest, uint16_t val) { dest = val; };

        // For these ops, n is a reg, imm, or mem val. 
        // Result stored in A.
        void add8(uint8_t n);
        void addc8(uint8_t n);
        void sub8(uint8_t n);
        void subc8(uint8_t n);
        void and8(uint8_t n);
        void or8(uint8_t n);
        void xor8(uint8_t n);
        // Compare A with n.
        void cp8(uint8_t n); 
        // n is a reg or mem val.
        void inc8(uint8_t &n);
        void dec8(uint8_t &n);
        void swap8(uint8_t &n);
        // Rotates left or right.
        void rlc8(uint8_t &n);
        void rl8(uint8_t &n);
        void rrc8(uint8_t &n);
        void rr8(uint8_t &n);
        // Shifts left or right.
        void sla8(uint8_t &n);
        void sra8(uint8_t &n);

        // Add Reg16 to HL.
        void add16_hl(Reg16 r);
        // Add imm n to SP.
        void add16_sp(uint8_t n);

        void inc16(Reg16 &r) { r++; };
        void dec16(Reg16 &r) { r--; };

        // Complement A.
        void cpl();
        // Complement Carry flag.
        void ccf();
        // Set Carry flag.
        void scf();

        // Bit operations.
        void bit(Reg8 r, uint8_t b);
        void set(Reg8 &r, uint8_t b) { r |= 1 << b; };
        void res(Reg8 &r, uint8_t b) { r &= ~(1 << b); };
    
        // Jumps.
        void jp(uint16_t addr);
        void jr(uint8_t offset);

        // Stack operations.
        void push(Reg16 r);
        void pop(Reg16 &r);
        void call(uint16_t addr);
        void ret();
};

#endif 
