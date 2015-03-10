#include "cpu.h"
#include <iostream>
#include <iomanip>

using std::cout;
using std::endl;
using std::setw;
using std::vector;


#define CC_TEST(CC,Y,F) ((Y) % 2 ? ((F) & CC[(Y)]) : !((F) & CC[(Y)]))
#define BIT(X, N) ((X) & (1 << (N)))


CPU::CPU() : op_table(build_op_table()), mem(65536)
{
    PC = 0x100;
    SP = 0xFFFE;

    interrupts = true;

    AF.set_high(0x69);

    uint8_t instr[] = { 0x38, 0x04, 0x31, 0x00, 0x80, 0x36, 0x05, 0x34, 0x34, 0x3a, 0x76 };
                        
    int n = sizeof(instr)/sizeof(*instr);
    vector<uint8_t> data(instr, instr + n);

    mem.load_bytes_to(0x100, data);

    int c = 0;
    while(mem[PC] != 0x76) {
        cout << std::hex << "0x" << (int) mem[PC] << std::dec << endl;
        op_table[mem[PC]]();
        PC++;
        c += cycles;
    }
    cout << "CYCLES: " << c << endl;
    mem.print(cout, 0x0000, 0x0001);

} // Constructor

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
            PC = PC + mem[++PC];
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
            PC = PC + mem[++PC];
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
            PC = PC + mem[++PC];
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
            PC = PC + mem[++PC];
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
    uint8_t flags = AF.low();
    flags &= ~FLAG_N;
    if(BIT(HL_old, 11) && BIT(r, 11)) 
        flags |= FLAG_H;
    if(BIT(HL_old, 15) && BIT(r, 15)) 
        flags |= FLAG_C;
    AF.set_low(flags);
} // add16_hl()

uint8_t CPU::inc8(uint8_t val)
{
    val++;

    uint8_t flags = AF.low();
    if(val == 0) 
        flags |= FLAG_Z;
    flags &= ~FLAG_N;
    if(~BIT(val - 1, 4) && BIT(val, 4))
        flags |= FLAG_H;
    AF.set_low(flags);

    return val;
} // inc8()

uint8_t CPU::dec8(uint8_t val) 
{
    val--;

    uint8_t flags = AF.low();
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
    uint8_t flags = AF.low();
    if(val == 0) 
        flags |= FLAG_Z;
    flags &= ~(FLAG_N | FLAG_H);
    msb ? flags |= FLAG_C : flags &= ~FLAG_C;
    AF.set_low(flags);

    return val;
} // rlc8()

uint8_t CPU::rrc8(uint8_t val)
{
    uint8_t lsb = BIT(val, 0);
    val = (val >> 1) + (lsb << 7);
    uint8_t flags = AF.low();
    if(val == 0) 
         flags|= FLAG_Z;
    flags &= ~(FLAG_N | FLAG_H);
    lsb ? flags |= FLAG_C : flags &= ~FLAG_C;

    return val;
} // rrc8()

uint8_t CPU::rl8(uint8_t val)
{
    uint8_t msb = BIT(val, 7);
    uint8_t flags = AF.low();
    val = (val << 1) + (flags  & FLAG_C ? 0x1 : 0x0);
    if(val == 0) 
        flags |= FLAG_Z;
    flags &= ~(FLAG_N | FLAG_H);
    msb ? flags |= FLAG_C : flags &= ~FLAG_C;
    
    return val;
} // rl8()

uint8_t CPU::rr8(uint8_t val)
{
    uint8_t lsb = BIT(val, 0);
    uint8_t flags = AF.low();
    val = (val >> 1) + (flags & FLAG_C ? 0x80 : 0x0);
    if(val == 0) 
        flags |= FLAG_Z;
    flags &= ~(FLAG_N | FLAG_H);
    lsb ? flags |= FLAG_C : flags &= ~FLAG_C;

    return val;
} // rr8()

/*

void CPU::add8(uint8_t n) 
{
    Reg8 &A = reg8[REG_A], &F = reg8[REG_F], A_old = reg8[REG_A];
    A += n;

    // Set flags
    F &= ~FLAG_N;
    if(A == 0)
        F |= FLAG_Z;
    if(BIT(A_old, 3) && BIT(n, 3)) 
        F |= FLAG_H;
    if(BIT(A_old, 7) && BIT(n, 7))
        F |= FLAG_C;
} // add8()

void CPU::addc8(uint8_t n)
{
    Reg8 F = reg8[REG_F];
    uint8_t carry = (F & FLAG_C) ? 0x1 : 0x0;
    add8(n + carry);
} // addc8()

void CPU::sub8(uint8_t n)
{
    Reg8 &A = reg8[REG_A], &F = reg8[REG_F], A_old = reg8[REG_A];
    A += ~n + 1;

    // Set flags
    F |= FLAG_N;
    if(A == 0)
        F |= FLAG_Z;
    if((A_old & 0x0f) < (n & 0x0f))
        F |= FLAG_H;
    if((A_old & 0xf0) < (n & 0xf0) 
      || (((A_old & 0xf0) == 0) && (F & FLAG_H)))
        F |= FLAG_C;
} // sub8()

void CPU::subc8(uint8_t n)
{
    Reg8 &F = reg8[REG_F];
    uint8_t carry = (F & FLAG_C) ? 0x1 : 0x0;
    sub8(~n + 1 + carry);
} // subc8()

void CPU::and8(uint8_t n)
{
    Reg8 &A = reg8[REG_A], &F = reg8[REG_F];
    A &= n;

    // Set flags
    if(A == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_C);
    F |= FLAG_H;
} // and8()

void CPU::or8(uint8_t n)
{
    Reg8 &A = reg8[REG_A], &F = reg8[REG_F];
    A |= n;

    // Set flags
    if(A == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H | FLAG_C);

} // or8()

void CPU::xor8(uint8_t n)
{
    Reg8 &A = reg8[REG_A], &F = reg8[REG_F];
    A ^= n;

    // Set flags
    if(A == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H | FLAG_C);

} // xor8()

void CPU::cp8(uint8_t n)
{
    // Save A b/c sub will modify.
    Reg8 A = reg8[REG_A];
    sub8(n);
    reg8[REG_A] = A; 
} // cp8()

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

void CPU::push(Reg16 r)
{
    sp--;
    mem[sp] = r & 0x00ff;
    sp--;
    mem[sp] = (r & 0xff00) >> 8;
} // push()

void CPU::pop(Reg16 &r)
{
    r = ((uint16_t)mem[sp]) << 8;
    sp++;
    r += (uint16_t)mem[sp];
    sp++;
} // pop()

void CPU::call(uint16_t addr)
{
    push(pc + 3);
    jp(addr);
} // call()

void CPU::ret()
{
    uint16_t addr;
    pop(addr);
    jp(addr);
} // ret()
*/
