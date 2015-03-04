#include "cpu.h"
#include <iostream>
#include <iomanip>
#include <bitset>
#include <algorithm>

using std::cout;
using std::endl;
using std::bitset;
using std::fill;
using std::copy;
using std::hex;
using std::dec;
using std::setw;
using std::left;

// Used in decoding tables.
const uint8_t MEM_HL = 255;
const uint8_t REG_SP = 255;

#define BIT16_FROM_MEM(A)  ((((uint16_t)(mem[(A)+1])) << 8) + mem[(A)])
#define CC_TEST(CC,Y,F) ((Y) % 2 ? ((F) & CC[(Y)]) : !((F) & CC[(Y)]))
#define BIT(X, N) ((X) & (1 << (N)))


CPU::CPU()
{
    pc = 0x100;
    sp = 0xFFFE;

    fill(reg8, reg8 + NUM_REG8, 0x0);

    fill(mem, mem + MEM_SIZE, 0x00);

    interrupts = true;

    uint8_t instr[] = { 0x3e, 0xff, 0x0e, 0x06, 
                        0xe2, 0xf0, 0x06,
                        0x08, 0x03, 0xff,
                        0xf8, 0xff, 0xf9,
                        0xcb, 0x36,
                        0xcb, 0x17,
                        0xcb, 0x17,
                        0xcb, 0xc0,
                        0x76 };
    uint8_t num_instr = sizeof(instr)/sizeof(*instr);

    copy(instr, instr + num_instr, &(mem[pc]));

    // A = max(A, B)
    //                   cp B  jr NC       ld A,B ret
    uint8_t instr2[] = { 0xb8, 0x30, 0x03, 0x78, 0xc9 };
    num_instr = sizeof(instr)/sizeof(*instr);

    copy(instr2, instr2 + num_instr, &(mem[0x8000]));

    while(mem[pc] != 0x76) {
        cout << hex << "PC=" << (int)pc << "  SP=" << (int)sp;
        cout << "  OP=" << (int)mem[pc] << dec << endl;
        fetch_decode_execute();
    }

    print_mem(0xff03, 0xff07);

} // Constructor

void CPU::print_registers() const
{
    cout << endl;
    cout << "PC = 0x" << hex << pc << dec << endl;
    cout << "SP = 0x" << hex << sp << dec << endl << endl; 
    for(int i = 0; i < NUM_REG8 / 2; i++) {
        print_register(2*i+1);
        print_register(2*i);
        cout << endl;
    }
} // print_registers()

void CPU::print_register(uint8_t reg_id) const
{
    static const Reg8 reg8_names[NUM_REG8] = 
        { 'F', 'A', 'C', 'B', 'E', 'D', 'L', 'H' };
    cout << "Reg " << reg8_names[reg_id]
         << " = " << (bitset<8>) reg8[reg_id] << endl;
} // print_register()

void CPU::print_mem(uint16_t addr) const 
{
    cout << "0x" << hex << addr << " = 0x" << (int)mem[addr] << dec << endl;
} // print_mem()

void CPU::print_mem(uint16_t begin, uint16_t end) const
{
    cout << hex;
    for(uint16_t i = begin, j; i != end; i = j) { 
        for(j = i; j != end && j < i + 4; j++) {
            cout << "0x" << j << " = 0x" << setw(2) << left 
                 << (int)mem[j] << "  ";
        }
        cout << endl;
    }
    cout << dec;
} // print_mem()

void CPU::fetch_decode_execute()
{
    uint8_t opcode = mem[pc];
    uint8_t x = opcode >> 6, y = (opcode & 0x38) >> 3, z = opcode & 0x07;
    uint8_t p = y >> 1, q = y % 2;

    Reg8 F = reg8[REG_F];

    // 8bit register table.
    static const uint8_t r[8] = { REG_B, REG_C, REG_D, REG_E, REG_H, 
                                  REG_L, MEM_HL, REG_A };
    // Register pairs featuring SP.
    static const uint8_t rp[4] = { REG_BC, REG_DE, REG_HL, REG_SP };
    // Register pairs featuring AF.
    static const uint8_t rp2[4] = { REG_BC, REG_DE, REG_HL, REG_AF };
    // Conditions
    static const uint8_t cc[4] = { FLAG_Z, FLAG_Z, FLAG_C, FLAG_C };
    // Arithmetic and logic operations.
    typedef void (CPU::*ALUOp)(uint8_t);
    static const ALUOp alu[8] = { &CPU::add8, &CPU::addc8, &CPU::sub8, 
                                  &CPU::subc8, &CPU::and8, &CPU::xor8,
                                  &CPU::or8, &CPU::cp8 };
    // Rotate and shift operations.
    typedef void (CPU::*RotOp)(uint8_t&);
    static const RotOp rot[8] = { &CPU::rlc8, &CPU::rrc8, &CPU::rl8, 
                                  &CPU::rr8, &CPU::sla8, &CPU::sra8,
                                  &CPU::srl8, &CPU::srl8 };

    if(opcode == 0xcb) { // Handle-CB prefixed opcodes
        pc++;
        opcode = mem[pc];
        x = opcode >> 6, y = (opcode & 0x38) >> 3, z = opcode & 0x07;
        switch(x) {
            case 0x0:
                if(y == 0x6) {
                    swap8(reg8[r[z]]);
                } else {
                    (this->*rot[y])(reg8[r[z]]);
                }
                pc++;
                break;
            case 0x1: // Test bit
                bit(reg8[r[z]], y);
                pc++;
                break;
            case 0x2: // Reset bit
                res(reg8[r[z]], y);
                pc++;
                break;
            case 0x3: // Set bit
                set(reg8[r[z]], y);
                pc++;
                break;
        }
        return;
    }

    switch(x) {
        case 0x0:
            switch(z) {
                case 0x0: 
                    switch(y) {
                        case 0x0: // NOP
                            pc++;
                            break;
                        case 0x1: // LD (nn),SP
                            mem[BIT16_FROM_MEM(pc+1)] = sp & 0x00ff;
                            mem[BIT16_FROM_MEM(pc+1)+1] = (sp & 0xff00)>>8;
                            pc += 3;
                            break;
                        case 0x2: // TODO: STOP
                            while(true);
                            pc++;
                            break;
                        case 0x3: // Jump relative
                            jr(mem[pc+1]);
                            break;
                        case 0x4:
                        case 0x5:
                        case 0x6:
                        case 0x7: // Jump relative conditional
                            if(CC_TEST(cc,y-4,F))
                                jr(mem[pc+1]);
                            else
                                pc += 2;
                            break;
                    }
                    break;
                case 0x1:
                    if(q == 0x0) { // 16bit load imm 
                        uint16_t imm = BIT16_FROM_MEM(pc+1);
                        if(rp[p] == REG_SP) 
                            sp = imm;
                        else 
                            reg16(rp[p]) = imm;
                        pc += 3;
                    } else { // Add reg16 to HL
                        if(rp[p] == REG_SP) 
                            add16_hl(sp);
                        else 
                            add16_hl(reg16(rp[p]));
                        pc++;
                    }
                    break;
                case 0x2: // Indirect loading
                    if(q == 0x0) {
                        switch(p) {
                            case 0x0:
                                mem[reg16(REG_BC)] = reg8[REG_A];
                                pc++;
                                break;
                            case 0x1:
                                mem[reg16(REG_BC)] = reg8[REG_A];
                                pc++;
                                break;
                            case 0x2: // LDI (HL),A
                                mem[reg16(REG_HL)] = reg8[REG_A];
                                inc16(reg16(REG_HL));
                                pc++;
                                break;
                            case 0x3: // LDD (HL),A
                                mem[reg16(REG_HL)] = reg8[REG_A];
                                dec16(reg16(REG_HL));
                                pc++;
                                break;
                        }
                    } else {
                        switch(p) {
                            case 0x0:
                                reg8[REG_A] = mem[reg16(REG_BC)];
                                pc++;
                                break;
                            case 0x1:
                                reg8[REG_A] = mem[reg16(REG_DE)];
                                pc++;
                                break;
                            case 0x2: // LDI A,(HL)
                                reg8[REG_A] = mem[reg16(REG_HL)];
                                inc16(reg16(REG_HL));
                                pc++;
                                break;
                            case 0x3: // LDD A,(HL)
                                reg8[REG_A] = mem[reg16(REG_HL)];
                                dec16(reg16(REG_HL));
                                pc++;
                                break;
                        }
                    }
                    break;
                case 0x3: 
                    if(q == 0x0) { // 16bit inc
                        if(rp[p] == REG_SP)
                            inc16(sp);
                        else 
                            inc16(reg16(rp[p]));
                    } else { // 16bit dec
                        if(rp[p] == REG_SP)
                            dec16(sp);
                        else 
                            dec16(reg16(rp[p]));
                    }
                    pc++;
                    break;
                case 0x4: // 8bit inc
                    if(r[y] == MEM_HL) 
                        inc8(mem[reg16(REG_HL)]);
                    else 
                        inc8(reg8[r[y]]);
                    pc++;
                    break;
                case 0x5: // 8bit dec 
                    if(r[y] == MEM_HL) 
                        dec8(mem[reg16(REG_HL)]);
                    else 
                        dec8(reg8[r[y]]);
                    pc++;
                    break;
                case 0x6: // 8bit load imm
                    if(r[y] == MEM_HL) 
                        mem[reg16(REG_HL)] = mem[pc+1];
                    else 
                        reg8[r[y]] = mem[pc+1];
                    pc += 2;
                    break;
                case 0x7: 
                    switch(y) {
                        case 0x0: // Rotate A left to carry
                            rlc8(reg8[REG_A]);
                            pc++;
                            break;
                        case 0x1: // Rotate A right to carry
                            rrc8(reg8[REG_A]);
                            pc++;
                            break;
                        case 0x2: // Rotate A left through carry
                            rl8(reg8[REG_A]) ;
                            pc++;
                            break;
                        case 0x3: // Rotate A right through carry
                            rr8(reg8[REG_A]);
                            pc++;
                            break;
                        case 0x4: // TODO: DAA
                            pc++;
                            break;
                        case 0x5: // Complement A
                            cpl();
                            pc++;
                            break;
                        case 0x6: // Set carry flag
                            scf();
                            pc++;
                            break;
                        case 0x7: // Complement carry flag
                            ccf();
                            pc++;
                            break;
                    }
            }
            break;
        case 0x1: 
            if(z == 0x6 && y == 0x6) {
                // TODO: HALT TEMP
                while(true);
            }
            // 8bit load of reg/mem
            if(r[y] == MEM_HL) 
                mem[reg16(REG_HL)] = reg8[r[z]];
            else if(r[z] == MEM_HL)
                reg8[r[y]] = mem[reg16(REG_HL)];
            else 
                reg8[r[y]] = reg8[r[z]];
            pc++;
            break;
        case 0x2:
            // ALU operation on reg/mem
            if(r[z] == MEM_HL) 
                (this->*alu[y])(mem[reg16(REG_HL)]);
            else 
                (this->*alu[y])(reg8[r[z]]);
            pc++;
            break;
        case 0x3:
            switch(z) {
                case 0x0: 
                    switch(y) {
                        case 0x0: // Conditional return
                            if(CC_TEST(cc,y,F))
                                ret();
                            else 
                                pc++;
                            break;
                        case 0x4: 
                            mem[0xff00 + mem[pc+1]] = reg8[REG_A];
                            pc += 2;
                            break;
                        case 0x5: 
                            add16_sp(mem[pc+1]);
                            pc += 2;
                            break;
                        case 0x6:
                            reg8[REG_A] = mem[0xff00 + mem[pc+1]];
                            pc += 2;
                            break;
                        case 0x7: // LD HL,SP+n
                            reg16(REG_HL) = sp + mem[pc+1];
                            pc += 2;
                            break;
                    }
                    break;
                case 0x1: 
                    if(q == 0x0) { // Pop
                        pop(reg16(rp2[p]));
                        pc++;
                    } else {
                        switch(p) {
                            case 0x0: 
                                ret();
                                break;
                            case 0x1:
                                ret();
                                interrupts = true;
                                break;
                            case 0x2:
                                jp(reg16(REG_HL));
                                break;
                            case 0x3:
                                sp = reg16(REG_HL);
                                pc++;
                                break;
                        }
                    }
                    break;
                case 0x2: 
                    switch(y) {
                        case 0x0:
                        case 0x1:
                        case 0x2:
                        case 0x3:
                            // Conditional jump
                            if(CC_TEST(cc,y,F))
                                jp(BIT16_FROM_MEM(pc+1));
                            else 
                                pc += 3;
                            break;
                        case 0x4:
                            mem[0xff00 + reg8[REG_C]] = reg8[REG_A];
                            pc++;
                            break;
                        case 0x6:
                            reg8[REG_A] = mem[0xff00 + reg8[REG_C]];
                            pc++;
                            break;
                    }
                    break;
                case 0x3: 
                    switch(y) {
                        case 0x0: // Jump
                            jp(BIT16_FROM_MEM(pc+1));
                            break;
                        case 0x6: 
                            interrupts = false;
                            pc++;
                            break;
                        case 0x7:
                            interrupts = true;
                            pc++;
                            break;
                    }
                    break;
                case 0x4: // Conditional call
                    if(CC_TEST(cc,y,F)) 
                        call(BIT16_FROM_MEM(pc+1));
                    else 
                        pc += 3;
                    break;
                case 0x5: 
                    if(q == 0x0) { // Push
                        push(reg16(rp2[p]));
                        pc++;
                    } else {  // Call
                        call(BIT16_FROM_MEM(pc+1));
                    }
                    break;
                case 0x6: // ALU op with imm
                    (this->*alu[y])(mem[pc+1]);
                    pc += 2;
                    break;
                case 0x7: // Restart
                    push(pc + 3);
                    jp(y*8);
                    break;
            }
            break;
    }
} // fetch_decode_execute()

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

void CPU::inc8(uint8_t &n)
{
    n++;

    Reg8 &F = reg8[REG_F];
    if(n == 0) 
        F |= FLAG_Z;
    F &= ~FLAG_N;
    if(~BIT(n - 1, 4) && BIT(n, 4))
        F |= FLAG_H;
} // inc8()

void CPU::dec8(uint8_t &n)
{
    n--;

    Reg8 &F = reg8[REG_F];
    if(n == 0) 
        F |= FLAG_Z;
    F |= FLAG_N;
    if(~BIT(n - 1, 4) && BIT(n, 4))
        F |= FLAG_H;
} // dec8()

void CPU::add16_hl(Reg16 r)
{
    Reg16 &HL = reg16(REG_HL), HL_old = reg16(REG_HL);
    HL += r;

    Reg8 &F = reg8[REG_F];
    F &= ~FLAG_N;
    if(BIT(HL_old, 11) && BIT(r, 11)) 
        F |= FLAG_H;
    if(BIT(HL_old, 15) && BIT(r, 15)) 
        F |= FLAG_C;
} // add16_hl()

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

void CPU::rl8(uint8_t &n)
{
    uint8_t msb = BIT(n, 7);
    Reg8 &F = reg8[REG_F];
    n = (n << 1) + (F & FLAG_C ? 0x1 : 0x0);
    if(n == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H);
    msb ? F |= FLAG_C : F &= ~FLAG_C;
} // rl8()

void CPU::rrc8(uint8_t &n)
{
    uint8_t lsb = BIT(n, 0);
    n = (n >> 1) + (lsb << 7);
    Reg8 &F = reg8[REG_F];
    if(n == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H);
    lsb ? F |= FLAG_C : F &= ~FLAG_C;
} // rrc8()

void CPU::rr8(uint8_t &n)
{
    uint8_t lsb = BIT(n, 0);
    Reg8 &F = reg8[REG_F];
    n = (n >> 1) + (F & FLAG_C ? 0x80 : 0x0);
    if(n == 0) 
        F |= FLAG_Z;
    F &= ~(FLAG_N | FLAG_H);
    lsb ? F |= FLAG_C : F &= ~FLAG_C;
} // rr8()

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

void CPU::jr(uint8_t offset)
{
    pc += offset;
} // jr()

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
