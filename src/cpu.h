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

        void run();

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

        uint8_t get_flags() const { return AF.low(); };
        void set_flags(uint8_t f) { AF.set_low(f); };

    //
    // Instruction Functions
    //

        void handle_cb_op();

        void load16_imm(Register&);
        void add16_hl(Register&);
        void add16_sp(uint8_t);
        void add8(uint8_t);
        void addc8(uint8_t);
        void sub8(uint8_t);
        void subc8(uint8_t);
        void and8(uint8_t);
        void or8(uint8_t);
        void xor8(uint8_t);
        void cp8(uint8_t); 
        uint8_t inc8(uint8_t);
        uint8_t dec8(uint8_t);

        void push(uint16_t);
        void pop(uint16_t&);
        void call(uint16_t);
        void ret();

        void daa();

        uint8_t swap8(uint8_t);
        uint8_t rlc8(uint8_t);
        uint8_t rrc8(uint8_t);
        uint8_t rl8(uint8_t);
        uint8_t rr8(uint8_t);
        uint8_t sla8(uint8_t);
        uint8_t sra8(uint8_t);
        uint8_t srl8(uint8_t);

};

#endif 
