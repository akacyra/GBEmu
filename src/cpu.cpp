#include "cpu.h"
#include <iostream>
#include <iomanip>
#include <fstream>

using std::cout;
using std::endl;
using std::setw;
using std::vector;


#define BIT(X, N) ((X) & (1 << (N)))

CPU::CPU() : op_table(build_op_table()), mem(65536)
{
    PC = 0x100;
    SP = 0xFFFE;

    interrupts = true;

    std::ifstream in("src/code", std::ios::binary | std::ios::in);
    vector<uint8_t> bytes;
    char byte;

    while(!in.eof()) {
        in.read(&byte, 1);
        bytes.push_back(byte);
    }

    in.close();

    mem.load_bytes_to(0x100, bytes);

} // Constructor

void CPU::run() 
{
    while(mem[PC] != 0x76) {
        cout << std::hex << (unsigned int)mem[PC] << std::dec << endl;
        op_table[mem[PC]]();
        PC++;
    }
} // run()

void CPU::print_registers() const
{
    cout << "PC -> " << setw(2) << PC << "    SP -> " << setw(2) << SP << endl; 
    cout << "AF -> " << setw(2) << AF << "    BC -> " << setw(2) << BC << endl; 
    cout << "DE -> " << setw(2) << DE << "    HL -> " << setw(2) << HL << endl; 
} // print_registers()

const vector<CPU::Instruction> CPU::build_op_table() 
{
    vector<Instruction> ops(256);
    ops[0x00] = [this]() { // NOP 
        cycles = 4;
    };
    ops[0x01] = [this]() { // LD BC,nn
        load16_imm(BC);
        cycles = 12;
    };
    ops[0x02] = [this]() { // LD (BC),A
        mem[BC] = AF.high(); 
        cycles = 8;
    };
    ops[0x03] = [this]() { // INC BC
        BC++;
        cycles = 8;
    };
    ops[0x04] = [this]() { // INC B
        BC.set_high(inc8(BC.high()));
        cycles = 4;
    };
    ops[0x05] = [this]() { // DEC B
        BC.set_high(dec8(BC.high()));
        cycles = 4;
    };
    ops[0x06] = [this]() { // LD B,n
        BC.set_high(mem[++PC]);
        cycles = 8;
    };
    ops[0x07] = [this]() { // RLCA
        AF.set_high(rlc8(AF.high()));
        cycles = 4;
    };
    ops[0x08] = [this]() { // LD (nn),SP
        mem.set16(mem.get16(++PC), SP);
        PC++;
        cycles = 20;
    };
    ops[0x09] = [this]() { // ADD HL,BC
        add16_hl(BC);
        cycles = 8;
    };
    ops[0x0a] = [this]() { // LD A,(BC)
        AF.set_high(mem[BC]);
        cycles = 8;
    };
    ops[0x0b] = [this]() { // DEC BC
        BC--;
        cycles = 8;
    };
    ops[0x0c] = [this]() { // INC C
        BC.set_low(inc8(BC.low()));
        cycles = 4;
    };
    ops[0x0d] = [this]() { // DEC C
        BC.set_low(dec8(BC.low()));
        cycles = 4;
    };
    ops[0x0e] = [this]() { // LD C,n
        BC.set_low(mem[++PC]);
        cycles = 8;
    };
    ops[0x0f] = [this]() { // RRCA
        AF.set_high(rrc8(AF.high()));
        cycles = 4;
    };
    ops[0x10] = [this]() { // STOP
        while(true) // TODO: Implement STOP
        cycles = 4;
    };
    ops[0x11] = [this]() { // LD DE,nn
        load16_imm(DE);
        cycles = 12;
    };
    ops[0x12] = [this]() { // LD (DE),A
        mem[DE] = AF.high(); 
        cycles = 8;
    };
    ops[0x13] = [this]() { // INC DE
        DE++;
        cycles = 8;
    };
    ops[0x14] = [this]() { // INC D
        DE.set_high(inc8(DE.high()));
        cycles = 4;
    };
    ops[0x15] = [this]() { // DEC D
        DE.set_high(dec8(DE.high()));
        cycles = 4;
    };
    ops[0x16] = [this]() { // LD D,n
        DE.set_high(mem[++PC]);
        cycles = 8;
    };
    ops[0x17] = [this]() { // RLA
        AF.set_high(rl8(AF.high()));
        cycles = 4;
    };
    ops[0x18] = [this]() { // JR n
        PC = PC + mem[++PC];
        cycles = 12;
    };
    ops[0x19] = [this]() { // ADD HL,DE
        add16_hl(DE);
        cycles = 8;
    };
    ops[0x1a] = [this]() { // LD A,(DE)
        AF.set_high(mem[DE]);
        cycles = 8;
    };
    ops[0x1b] = [this]() { // DEC DE
        DE--;
        cycles = 8;
    };
    ops[0x1c] = [this]() { // INC E
        DE.set_low(inc8(DE.low()));
        cycles = 4;
    };
    ops[0x1d] = [this]() { // DEC E
        DE.set_low(dec8(DE.low()));
        cycles = 4;
    };
    ops[0x1e] = [this]() { // LD E,n
        DE.set_low(mem[++PC]);
        cycles = 8;
    };
    ops[0x1f] = [this]() { // RRA
        AF.set_high(rr8(AF.high()));
        cycles = 4;
    };
    ops[0x20] = [this]() { // JR NZ,n
        if(!(AF.low() & FLAG_Z)) {
            PC = PC + mem[PC+1];
            cycles = 12;
        } else {
            PC++;
            cycles = 8;
        }
    };
    ops[0x21] = [this]() { // LD HL,nn
        load16_imm(HL);
        cycles = 12;
    };
    ops[0x22] = [this]() { // LD (HL+),A
        mem[HL] = AF.high(); HL++;
        cycles = 8;
    };
    ops[0x23] = [this]() { // INC HL
        HL++;
        cycles = 8;
    };
    ops[0x24] = [this]() { // INC H
        HL.set_high(inc8(HL.high()));
        cycles = 4;
    };
    ops[0x25] = [this]() { // DEC H
        HL.set_high(dec8(HL.high()));
        cycles = 4;
    };
    ops[0x26] = [this]() { // LD H,n
        HL.set_high(mem[++PC]);
        cycles = 8;
    };
    ops[0x27] = [this]() { // DAA
        // TODO: Implement DAA
        cycles = 4;
    };
    ops[0x28] = [this]() { // JR Z,n
        if(AF.low() & FLAG_Z) {
            PC = PC + mem[PC+1];
            cycles = 12;
        } else {
            PC++;
            cycles = 8;
        }
    };
    ops[0x29] = [this]() { // ADD HL,HL
        add16_hl(HL);
        cycles = 8;
    };
    ops[0x2a] = [this]() { // LD A,(HL+)
        AF.set_high(mem[HL]); HL++;
        cycles = 8;
    };
    ops[0x2b] = [this]() { // DEC HL
        HL--;
        cycles = 8;
    };
    ops[0x2c] = [this]() { // INC L
        HL.set_low(inc8(HL.low()));
        cycles = 4;
    };
    ops[0x2d] = [this]() { // DEC L
        HL.set_low(dec8(HL.low()));
        cycles = 4;
    };
    ops[0x2e] = [this]() { // LD L,n
        HL.set_low(mem[++PC]);
        cycles = 8;
    };
    ops[0x2f] = [this]() { // CPL
        AF.set_high(~AF.high());
        AF.set_low(AF.low() | FLAG_N | FLAG_H);
        cycles = 4;
    };
    ops[0x30] = [this]() { // JR NC,n
        if(!(AF.low() & FLAG_C)) {
            PC = PC + mem[PC+1];
            cycles = 12;
        } else {
            PC++;
            cycles = 8;
        }
    };
    ops[0x31] = [this]() { // LD SP,nn
        load16_imm(SP);
        cycles = 12;
    };
    ops[0x32] = [this]() { // LD (HL-),A
        mem[HL] = AF.high(); HL--;
        cycles = 8;
    };
    ops[0x33] = [this]() { // INC SP
        SP++;
        cycles = 8;
    };
    ops[0x34] = [this]() { // INC (HL)
        mem[HL] = inc8(mem[HL]);
        cycles = 12;
    };
    ops[0x35] = [this]() { // DEC (HL) 
        mem[HL] = dec8(mem[HL]);
        cycles = 12;
    };
    ops[0x36] = [this]() { // LD (HL),n
        mem[HL] = mem[++PC];
        cycles = 12;
    };
    ops[0x37] = [this]() { // SCF
        AF.set_low(AF.low() | FLAG_C);
        AF.set_low(AF.low() & ~(FLAG_N | FLAG_H));
        cycles = 4;
    };
    ops[0x38] = [this]() { // JR C,n
        if(AF.low() & FLAG_C) {
            PC = PC + mem[PC+1];
            cycles = 12;
        } else {
            PC++;
            cycles = 8;
        }
    };
    ops[0x39] = [this]() { // ADD HL,SP
        add16_hl(SP);
        cycles = 8;
    };
    ops[0x3a] = [this]() { // LD A,(HL-)
        AF.set_high(mem[HL]); HL--;
        cycles = 8;
    };
    ops[0x3b] = [this]() { // DEC SP
        SP--;
        cycles = 8;
    };
    ops[0x3c] = [this]() { // INC A
        AF.set_high(inc8(AF.high()));
        cycles = 4;
    };
    ops[0x3d] = [this]() { // DEC A
        AF.set_high(dec8(AF.high()));
        cycles = 4;
    };
    ops[0x3e] = [this]() { // LD A,n
        AF.set_high(mem[++PC]);
        cycles = 8;
    };
    ops[0x3f] = [this]() { // CCF
        AF.set_low(AF.low() & FLAG_C ? AF.low() & ~FLAG_C : 
                                       AF.low() |  FLAG_C);
        AF.set_low(AF.low() & ~(FLAG_N | FLAG_H));
        cycles = 4;
    };
    ops[0x40] = [this]() { // LD B,B
        BC.set_high(BC.high());
        cycles = 4;
    };
    ops[0x41] = [this]() { // LD B,C
        BC.set_high(BC.low());
        cycles = 4;
    };
    ops[0x42] = [this]() { // LD B,D
        BC.set_high(DE.high());
        cycles = 4;
    };
    ops[0x43] = [this]() { // LD B,E
        BC.set_high(DE.low());
        cycles = 4;
    };
    ops[0x44] = [this]() { // LD B,H
        BC.set_high(HL.high());
        cycles = 4;
    };
    ops[0x45] = [this]() { // LD B,L
        BC.set_high(HL.low());
        cycles = 4;
    };
    ops[0x46] = [this]() { // LD B,(HL)
        BC.set_high(mem[HL]);
        cycles = 8;
    };
    ops[0x47] = [this]() { // LD B,A
        BC.set_high(AF.high());
        cycles = 4;
    };
    ops[0x48] = [this]() { // LD C,B
        BC.set_low(BC.high());
        cycles = 4;
    };
    ops[0x49] = [this]() { // LD C,C
        BC.set_low(BC.low());
        cycles = 4;
    };
    ops[0x4a] = [this]() { // LD C,D
        BC.set_low(DE.high());
        cycles = 4;
    };
    ops[0x4b] = [this]() { // LD C,E
        BC.set_low(DE.low());
        cycles = 4;
    };
    ops[0x4c] = [this]() { // LD C,H
        BC.set_low(HL.high());
        cycles = 4;
    };
    ops[0x4d] = [this]() { // LD C,L
        BC.set_low(HL.low());
        cycles = 4;
    };
    ops[0x4e] = [this]() { // LD C,(HL)
        BC.set_low(mem[HL]);
        cycles = 8;
    };
    ops[0x4f] = [this]() { // LD C,A
        BC.set_low(AF.high());
        cycles = 4;
    };
    ops[0x50] = [this]() { // LD D,B
        DE.set_high(BC.high());
        cycles = 4;
    };
    ops[0x51] = [this]() { // LD D,C
        DE.set_high(BC.low());
        cycles = 4;
    };
    ops[0x52] = [this]() { // LD D,D
        DE.set_high(DE.high());
        cycles = 4;
    };
    ops[0x53] = [this]() { // LD D,E
        DE.set_high(DE.low());
        cycles = 4;
    };
    ops[0x54] = [this]() { // LD D,H
        DE.set_high(HL.high());
        cycles = 4;
    };
    ops[0x55] = [this]() { // LD D,L
        DE.set_high(HL.low());
        cycles = 4;
    };
    ops[0x56] = [this]() { // LD D,(HL)
        DE.set_high(mem[HL]);
        cycles = 8;
    };
    ops[0x57] = [this]() { // LD D,A
        DE.set_high(AF.high());
        cycles = 4;
    };
    ops[0x58] = [this]() { // LD E,B
        DE.set_low(BC.high());
        cycles = 4;
    };
    ops[0x59] = [this]() { // LD E,C
        DE.set_low(BC.low());
        cycles = 4;
    };
    ops[0x5a] = [this]() { // LD E,D
        DE.set_low(DE.high());
        cycles = 4;
    };
    ops[0x5b] = [this]() { // LD E,E
        DE.set_low(DE.low());
        cycles = 4;
    };
    ops[0x5c] = [this]() { // LD E,H
        DE.set_low(HL.high());
        cycles = 4;
    };
    ops[0x5d] = [this]() { // LD E,L
        DE.set_low(HL.low());
        cycles = 4;
    };
    ops[0x5e] = [this]() { // LD E,(HL)
        DE.set_low(mem[HL]);
        cycles = 8;
    };
    ops[0x5f] = [this]() { // LD E,A
        DE.set_low(AF.high());
        cycles = 4;
    };
    ops[0x60] = [this]() { // LD H,B
        HL.set_high(BC.high());
        cycles = 4;
    };
    ops[0x61] = [this]() { // LD H,C
        HL.set_high(BC.low());
        cycles = 4;
    };
    ops[0x62] = [this]() { // LD H,D
        HL.set_high(DE.high());
        cycles = 4;
    };
    ops[0x63] = [this]() { // LD H,E
        HL.set_high(DE.low());
        cycles = 4;
    };
    ops[0x64] = [this]() { // LD H,H
        HL.set_high(HL.high());
        cycles = 4;
    };
    ops[0x65] = [this]() { // LD H,L
        HL.set_high(HL.low());
        cycles = 4;
    };
    ops[0x66] = [this]() { // LD H,(HL)
        HL.set_high(mem[HL]);
        cycles = 8;
    };
    ops[0x67] = [this]() { // LD H,A
        HL.set_high(AF.high());
        cycles = 4;
    };
    ops[0x68] = [this]() { // LD L,B
        HL.set_low(BC.high());
        cycles = 4;
    };
    ops[0x69] = [this]() { // LD L,C
        HL.set_low(BC.low());
        cycles = 4;
    };
    ops[0x6a] = [this]() { // LD L,D
        HL.set_low(DE.high());
        cycles = 4;
    };
    ops[0x6b] = [this]() { // LD L,E
        HL.set_low(DE.low());
        cycles = 4;
    };
    ops[0x6c] = [this]() { // LD L,H
        HL.set_low(HL.high());
        cycles = 4;
    };
    ops[0x6d] = [this]() { // LD L,L
        HL.set_low(HL.low());
        cycles = 4;
    };
    ops[0x6e] = [this]() { // LD L,(HL)
        HL.set_low(mem[HL]);
        cycles = 8;
    };
    ops[0x6f] = [this]() { // LD L,A
        HL.set_low(AF.high());
        cycles = 4;
    };
    ops[0x70] = [this]() { // LD (HL),B
        mem[HL] = BC.high();
        cycles = 8;
    };
    ops[0x71] = [this]() { // LD (HL),C
        mem[HL] = BC.low();
        cycles = 8;
    };
    ops[0x72] = [this]() { // LD (HL),D
        mem[HL] = DE.high();
        cycles = 8;
    };
    ops[0x73] = [this]() { // LD (HL),E
        mem[HL] = DE.low();
        cycles = 8;
    };
    ops[0x74] = [this]() { // LD (HL),H
        mem[HL] = HL.high();
        cycles = 8;
    };
    ops[0x75] = [this]() { // LD (HL),L
        mem[HL] = HL.low();
        cycles = 8;
    };
    ops[0x76] = [this]() { // HALT
        while(true); // TODO: Implement HALT
        cycles = 4;
    };
    ops[0x77] = [this]() { // LD (HL),A
        mem[HL] = AF.high();
        cycles = 8;
    };
    ops[0x78] = [this]() { // LD A,B
        AF.set_high(BC.high());
        cycles = 4;
    };
    ops[0x79] = [this]() { // LD A,C
        AF.set_high(BC.low());
        cycles = 4;
    };
    ops[0x7a] = [this]() { // LD A,D
        AF.set_high(DE.high());
        cycles = 4;
    };
    ops[0x7b] = [this]() { // LD A,E
        AF.set_high(DE.low());
        cycles = 4;
    };
    ops[0x7c] = [this]() { // LD A,H
        AF.set_high(HL.high());
        cycles = 4;
    };
    ops[0x7d] = [this]() { // LD A,L
        AF.set_high(HL.low());
        cycles = 4;
    };
    ops[0x7e] = [this]() { // LD A,(HL)
        AF.set_high(mem[HL]);
        cycles = 8;
    };
    ops[0x7f] = [this]() { // LD A,A
        AF.set_high(AF.high());
        cycles = 4;
    };
    ops[0x80] = [this]() { // ADD A,B
        add8(BC.high());
        cycles = 4;
    };
    ops[0x81] = [this]() { // ADD A,C
        add8(BC.low());
        cycles = 4;
    };
    ops[0x82] = [this]() { // ADD A,D
        add8(DE.high());
        cycles = 4;
    };
    ops[0x83] = [this]() { // ADD A,E
        add8(DE.low());
        cycles = 4;
    };
    ops[0x84] = [this]() { // ADD A,H
        add8(HL.high());
        cycles = 4;
    };
    ops[0x85] = [this]() { // ADD A,L
        add8(HL.low());
        cycles = 4;
    };
    ops[0x86] = [this]() { // ADD A,(HL)
        add8(mem[HL]);
        cycles = 8;
    };
    ops[0x87] = [this]() { // ADD A,A
        add8(AF.high());
        cycles = 4;
    };
    ops[0x88] = [this]() { // ADC A,B
        addc8(BC.high());
        cycles = 4;
    };
    ops[0x89] = [this]() { // ADC A,C
        addc8(BC.low());
        cycles = 4;
    };
    ops[0x8a] = [this]() { // ADC A,D
        addc8(DE.high());
        cycles = 4;
    };
    ops[0x8b] = [this]() { // ADC A,E
        addc8(DE.low());
        cycles = 4;
    };
    ops[0x8c] = [this]() { // ADC A,H
        addc8(HL.high());
        cycles = 4;
    };
    ops[0x8d] = [this]() { // ADC A,L
        addc8(HL.low());
        cycles = 4;
    };
    ops[0x8e] = [this]() { // ADC A,(HL)
        addc8(mem[HL]);
        cycles = 8;
    };
    ops[0x8f] = [this]() { // ADC A,A
        addc8(AF.high());
        cycles = 4;
    };
    ops[0x90] = [this]() { // SUB A,B
        sub8(BC.high());
        cycles = 4;
    };
    ops[0x91] = [this]() { // SUB A,C
        sub8(BC.low());
        cycles = 4;
    };
    ops[0x92] = [this]() { // SUB A,D
        sub8(DE.high());
        cycles = 4;
    };
    ops[0x93] = [this]() { // SUB A,E
        sub8(DE.low());
        cycles = 4;
    };
    ops[0x94] = [this]() { // SUB A,H
        sub8(HL.high());
        cycles = 4;
    };
    ops[0x95] = [this]() { // SUB A,L
        sub8(HL.low());
        cycles = 4;
    };
    ops[0x96] = [this]() { // SUB A,(HL)
        sub8(mem[HL]);
        cycles = 8;
    };
    ops[0x97] = [this]() { // SUB A,A
        sub8(AF.high());
        cycles = 4;
    };
    ops[0x98] = [this]() { // SBC A,B
        subc8(BC.high());
        cycles = 4;
    };
    ops[0x99] = [this]() { // SBC A,C
        subc8(BC.low());
        cycles = 4;
    };
    ops[0x9a] = [this]() { // SBC A,D
        subc8(DE.high());
        cycles = 4;
    };
    ops[0x9b] = [this]() { // SBC A,E
        subc8(DE.low());
        cycles = 4;
    };
    ops[0x9c] = [this]() { // SBC A,H
        subc8(HL.high());
        cycles = 4;
    };
    ops[0x9d] = [this]() { // SBC A,L
        subc8(HL.low());
        cycles = 4;
    };
    ops[0x9e] = [this]() { // SBC A,(HL)
        subc8(mem[HL]);
        cycles = 8;
    };
    ops[0x9f] = [this]() { // SBC A,A
        subc8(AF.high());
        cycles = 4;
    };
    ops[0xa0] = [this]() { // AND A,B
        and8(BC.high());
        cycles = 4;
    };
    ops[0xa1] = [this]() { // AND A,C
        and8(BC.low());
        cycles = 4;
    };
    ops[0xa2] = [this]() { // AND A,D
        and8(DE.high());
        cycles = 4;
    };
    ops[0xa3] = [this]() { // AND A,E
        and8(DE.low());
        cycles = 4;
    };
    ops[0xa4] = [this]() { // AND A,H
        and8(HL.high());
        cycles = 4;
    };
    ops[0xa5] = [this]() { // AND A,L
        and8(HL.low());
        cycles = 4;
    };
    ops[0xa6] = [this]() { // AND A,(HL)
        and8(mem[HL]);
        cycles = 8;
    };
    ops[0xa7] = [this]() { // AND A,A
        and8(AF.high());
        cycles = 4;
    };
    ops[0xa8] = [this]() { // XOR A,B
        xor8(BC.high());
        cycles = 4;
    };
    ops[0xa9] = [this]() { // XOR A,C
        xor8(BC.low());
        cycles = 4;
    };
    ops[0xaa] = [this]() { // XOR A,D
        xor8(DE.high());
        cycles = 4;
    };
    ops[0xab] = [this]() { // XOR A,E
        xor8(DE.low());
        cycles = 4;
    };
    ops[0xac] = [this]() { // XOR A,H
        xor8(HL.high());
        cycles = 4;
    };
    ops[0xad] = [this]() { // XOR A,L
        xor8(HL.low());
        cycles = 4;
    };
    ops[0xae] = [this]() { // XOR A,(HL)
        xor8(mem[HL]);
        cycles = 8;
    };
    ops[0xaf] = [this]() { // XOR A,A
        xor8(AF.high());
        cycles = 4;
    };
    ops[0xb0] = [this]() { // OR A,B
        or8(BC.high());
        cycles = 4;
    };
    ops[0xb1] = [this]() { // OR A,C
        or8(BC.low());
        cycles = 4;
    };
    ops[0xb2] = [this]() { // OR A,D
        or8(DE.high());
        cycles = 4;
    };
    ops[0xb3] = [this]() { // OR A,E
        or8(DE.low());
        cycles = 4;
    };
    ops[0xb4] = [this]() { // OR A,H
        or8(HL.high());
        cycles = 4;
    };
    ops[0xb5] = [this]() { // OR A,L
        or8(HL.low());
        cycles = 4;
    };
    ops[0xb6] = [this]() { // OR A,(HL)
        or8(mem[HL]);
        cycles = 8;
    };
    ops[0xb7] = [this]() { // OR A,A
        or8(AF.high());
        cycles = 4;
    };
    ops[0xb8] = [this]() { // CP A,B
        cp8(BC.high());
        cycles = 4;
    };
    ops[0xb9] = [this]() { // CP A,C
        cp8(BC.low());
        cycles = 4;
    };
    ops[0xba] = [this]() { // CP A,D
        cp8(DE.high());
        cycles = 4;
    };
    ops[0xbb] = [this]() { // CP A,E
        cp8(DE.low());
        cycles = 4;
    };
    ops[0xbc] = [this]() { // CP A,H
        cp8(HL.high());
        cycles = 4;
    };
    ops[0xbd] = [this]() { // CP A,L
        cp8(HL.low());
        cycles = 4;
    };
    ops[0xbe] = [this]() { // CP A,(HL)
        cp8(mem[HL]);
        cycles = 8;
    };
    ops[0xbf] = [this]() { // CP A,A
        cp8(AF.high());
        cycles = 4;
    };
    ops[0xc0] = [this]() { // RET NX
        if(!(AF.low() & FLAG_Z)) {
            ret();
            cycles = 20;
        } else 
            cycles = 8;
    };
    ops[0xc1] = [this]() { // POP BC
        uint16_t val;
        pop(val);
        BC = val;
        cycles = 12;
    };
    ops[0xc2] = [this]() { // JP NZ,nn
        if(!(AF.low() & FLAG_Z)) {
            PC = mem.get16(PC+1); 
            cycles = 16;
        } else {
            PC = PC + 2;
            cycles = 12;
        }
    };
    ops[0xc3] = [this]() { // JP nn
        PC = mem.get16(PC+1); 
        cycles = 16;
    };
    ops[0xc4] = [this]() { // CALL NZ,nn
        if(!(AF.low() & FLAG_Z)) {
            call(mem.get16(PC+1)); 
            cycles = 24;
        } else {
            PC = PC + 2;
            cycles = 12;
        }
    };
    ops[0xc5] = [this]() { // PUSH BC
        push(BC);
        cycles = 16;
    };
    ops[0xc6] = [this]() { // ADD A,n
        add8(mem[++PC]);
        cycles = 8;
    };
    ops[0xc7] = [this]() { // RST 00H
        call(0x00);
        cycles = 16;
    };
    ops[0xc8] = [this]() { // RET Z
        if(AF.low() & FLAG_Z) {
            ret();
            cycles = 20;
        } else 
            cycles = 8;
    };
    ops[0xc9] = [this]() { // RET
        ret();
        cycles = 16;
    };
    ops[0xca] = [this]() { // JP Z,nn
        if(AF.low() & FLAG_Z) {
            PC = mem.get16(PC+1); 
            cycles = 16;
        } else {
            PC = PC + 2;
            cycles = 12;
        }
    };
    ops[0xcb] = [this]() { // PREFIX CB
        // TODO: Handle CB-prefixed instructions.
    };
    ops[0xcc] = [this]() { // CALL Z,nn
        if(AF.low() & FLAG_Z) {
            call(mem.get16(PC+1)); 
            cycles = 24;
        } else {
            PC = PC + 2;
            cycles = 12;
        }
    };
    ops[0xcd] = [this]() { // CALL nn
        call(mem.get16(PC+1)); 
        cycles = 24;
    };
    ops[0xce] = [this]() { // ADC,n
        addc8(mem[++PC]);
        cycles = 8;
    };
    ops[0xcf] = [this]() { // RST 08H
        call(0x08);
        cycles = 16;
    };



    return ops;
} // build_op_table()

void CPU::load16_imm(Register &r)
{
    r.set_low(mem[++PC]);
    r.set_high(mem[++PC]);
} // load16_imm()

void CPU::add16_hl(Register& r)
{
    uint16_t HL_old = HL;
    HL = HL + r;
    uint8_t flags = AF.low() & FLAG_Z;
    if(BIT(HL_old, 11) && BIT(r, 11)) 
        flags |= FLAG_H;
    if(BIT(HL_old, 15) && BIT(r, 15)) 
        flags |= FLAG_C;
    AF.set_low(flags);
} // add16_hl()

void CPU::add8(uint8_t val) 
{
    uint8_t flags = 0, A_old = AF.high();
    AF.set_high(AF.high() + val);
    if(AF.high() == 0)
        flags |= FLAG_Z;
    if(BIT(A_old, 3) && BIT(val, 3)) 
        flags |= FLAG_H;
    if(BIT(A_old, 7) && BIT(val, 7))
        flags |= FLAG_C;
    AF.set_low(flags);
} // add8()

void CPU::addc8(uint8_t val)
{
    uint8_t carry = (AF.low() & FLAG_C) ? 0x1 : 0x0;
    add8(val + carry);
} // addc8()

void CPU::sub8(uint8_t val)
{
    uint8_t flags = 0, A_old = AF.high();
    AF.set_high(AF.high() + ~val + 1);
    flags  |= FLAG_N;
    if(AF.high() == 0)
        flags |= FLAG_Z;
    if((A_old & 0x0f) < (val & 0x0f))
        flags |= FLAG_H;
    if(A_old < val)
        flags |= FLAG_C;
    AF.set_low(flags);
} // sub8()

void CPU::subc8(uint8_t val)
{
    uint8_t carry = (AF.low() & FLAG_C) ? 0x1 : 0x0;
    sub8(val + carry);
} // subc8()

void CPU::and8(uint8_t val)
{
    AF.set_high(AF.high() & val);
    uint8_t flags = 0;
    if(AF.high() == 0) 
        flags |= FLAG_Z;
    flags |= FLAG_H;
    AF.set_low(flags);
} // and8()

void CPU::or8(uint8_t val)
{
    AF.set_high(AF.high() | val);
    uint8_t flags = 0;
    if(AF.high() == 0) 
        flags |= FLAG_Z;
    AF.set_low(flags);
} // or8()

void CPU::xor8(uint8_t val)
{
    AF.set_high(AF.high() ^ val);
    uint8_t flags = 0;
    if(AF.high() == 0) 
        flags |= FLAG_Z;
    AF.set_low(flags);
} // xor8()

void CPU::cp8(uint8_t val)
{
    // Save A because sub will modify it.
    uint8_t A = AF.high();
    sub8(val);
    AF.set_high(A);
} // cp8()

uint8_t CPU::inc8(uint8_t val)
{
    val++;
    uint8_t flags = AF.low() & FLAG_C;
    if(val == 0) 
        flags |= FLAG_Z;
    if(~BIT(val - 1, 4) && BIT(val, 4))
        flags |= FLAG_H;
    AF.set_low(flags);
    return val;
} // inc8()

uint8_t CPU::dec8(uint8_t val) 
{
    val--;
    uint8_t flags = AF.low() & FLAG_C;
    if(val == 0) 
        flags |= FLAG_Z;
    flags |= FLAG_N;
    if(~BIT(val - 1, 4) && BIT(val, 4))
        flags |= FLAG_H;
    AF.set_low(flags);
    return val;
} // dec8()

uint8_t CPU::rlc8(uint8_t val)
{
    uint8_t msb = BIT(val, 7);
    val = (val << 1) + (msb >> 7);
    uint8_t flags = 0;
    if(val == 0) 
        flags |= FLAG_Z;
    msb ? flags |= FLAG_C : flags &= ~FLAG_C;
    AF.set_low(flags);
    return val;
} // rlc8()

uint8_t CPU::rrc8(uint8_t val)
{
    uint8_t lsb = BIT(val, 0);
    val = (val >> 1) + (lsb << 7);
    uint8_t flags = 0;
    if(val == 0) 
         flags|= FLAG_Z;
    lsb ? flags |= FLAG_C : flags &= ~FLAG_C;

    return val;
} // rrc8()

uint8_t CPU::rl8(uint8_t val)
{
    uint8_t msb = BIT(val, 7);
    val = (val << 1) + (AF.low() & FLAG_C ? 0x1 : 0x0);
    uint8_t flags = 0;
    if(val == 0) 
        flags |= FLAG_Z;
    msb ? flags |= FLAG_C : flags &= ~FLAG_C;
    return val;
} // rl8()

uint8_t CPU::rr8(uint8_t val)
{
    uint8_t lsb = BIT(val, 0);
    val = (val >> 1) + (AF.low() & FLAG_C ? 0x80 : 0x0);
    uint8_t flags = 0;
    if(val == 0) 
        flags |= FLAG_Z;
    lsb ? flags |= FLAG_C : flags &= ~FLAG_C;

    return val;
} // rr8()

void CPU::push(uint16_t val)
{
    SP = SP - 2;
    mem.set16(SP, val);
} // push()

void CPU::pop(uint16_t& val)
{
    val = mem.get16(SP);
    SP = SP + 2;
} // pop()

void CPU::call(uint16_t addr)
{
    push(PC + 3);
    PC = addr;
} // call()

void CPU::ret()
{
    uint16_t addr;
    pop(addr);
    PC = addr;
} // ret()

/*



void CPU::add16_sp(uint8_t n)
{
    sp += n;

    Reg8 &F = reg8[REG_F];
    F &= ~FLAG_Z;
    F &= ~FLAG_N;
    // TODO: Flags H and C???
} // add16_sp()

void CPU::swap8(uint8_t &n)
{
    uint8_t temp = n >> 4;
    n <<= 4;
    n |= temp;

    Reg8 &F = reg8[REG_F];
    if(n == 0)
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H | FLAG_C);
} // swap8()

void CPU::cpl()
{
    reg8[REG_A] = ~reg8[REG_A];

    Reg8 &F = reg8[REG_F];
    F |= (FLAG_N | FLAG_H);
} // cpl()

void CPU::ccf()
{
    Reg8 &F = reg8[REG_F];
    F &= ~(FLAG_N | FLAG_H);
    if(F & FLAG_C) 
        F &= ~FLAG_C;
    else 
        F |= FLAG_C;
} // ccf()

void CPU::scf()
{
    Reg8 &F = reg8[REG_F];
    F &= ~(FLAG_N | FLAG_H);
    F |= FLAG_C;
} // scf()

void CPU::bit(Reg8 r, uint8_t b)
{
    Reg8 &F = reg8[REG_F];
    if(BIT(r, b))
        F &= ~FLAG_Z;
    else 
        F |= FLAG_Z;
    F &= ~FLAG_N;
    F |= FLAG_H;
} // bit()

void CPU::rlc8(uint8_t &n)
{
    uint8_t msb = BIT(n, 7);
    n = (n << 1) + (msb >> 7);
    Reg8 &F = reg8[REG_F];
    if(n == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H);
    msb ? F |= FLAG_C : F &= ~FLAG_C;
} // rlc8()





void CPU::sla8(uint8_t &n)
{
    uint8_t msb = BIT(n, 7);
    n <<= 1;
    Reg8 &F = reg8[REG_F];
    if(n == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H);
    msb ? F |= FLAG_C : F &= ~FLAG_C;
} // sla8()

void CPU::sra8(uint8_t &n)
{
    uint8_t lsb = BIT(n, 0);
    n = (n >> 1) + BIT(n, 7);
    Reg8 &F = reg8[REG_F];
    if(n == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H);
    lsb ? F |= FLAG_C : F &= ~FLAG_C;
} // sra8()

void CPU::srl8(uint8_t &n)
{
    uint8_t msb = BIT(n, 7);
    n = n >> 1;
    Reg8 &F = reg8[REG_F];
    if(n == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H);
    msb ? F |= FLAG_C : F &= ~FLAG_C;
} // srl8()

void CPU::jp(uint16_t addr)
{
    pc = addr;
} // jp()

*/
