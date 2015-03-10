#ifndef GUARD_CPU_H
#define GUARD_CPU_H

#include <vector>
#include <stdint.h>
#include <functional>
#include "memory.h"
#include "register.h"


// Zero Flag
const uint8_t FLAG_Z = 0x80;
// Subtract Flag
const uint8_t FLAG_N = 0x40;
// Half Carry Flag
const uint8_t FLAG_H = 0x20;
// Carry Flag 
const uint8_t FLAG_C = 0x10;

class CPU {
    public:
        CPU();

        void print_registers() const;

    private:
        typedef std::function<void()> Instruction;

        // Instruction table
        const std::vector<Instruction> op_table;
        const std::vector<Instruction> build_op_table();

        // Program Counter
        Register PC; 
        // Stack Pointer 
        Register SP; 
        // 8 bit registers which form 16 bit register pairs.
        Register AF, BC, DE, HL;

        Memory mem;

        uint8_t cycles;

        bool interrupts;

    //
    // Instruction Functions
    //

        void load16_imm(Register&);
        void add16_hl(Register&);
        uint8_t inc8(uint8_t);
        uint8_t dec8(uint8_t);
        uint8_t rlc8(uint8_t);
        uint8_t rrc8(uint8_t);
        uint8_t rl8(uint8_t);
        uint8_t rr8(uint8_t);

        /*
        void load16_imm();

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
        void rr8(uint8_t &n);
        // Shifts left or right.
        void sla8(uint8_t &n);
        void sra8(uint8_t &n);
        void srl8(uint8_t &n);

        // Add Reg16 to HL.
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
        */
};

#endif 
