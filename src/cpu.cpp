#include "cpu.h"
#include <iostream>
#include <iomanip>
#include <bitset>
#include <algorithm>

using std::cout;
using std::endl;
using std::bitset;
using std::fill;
using std::hex;
using std::dec;
using std::setw;
using std::left;

CPU::CPU()
{
    pc = 0x100;
    sp = 0xFFFE;

    fill(reg8, reg8 + NUM_REG8, 0x0);

    fill(mem, mem + MEM_SIZE, 0x00);

    interrupts = true;


    ld8(reg8[REG_A], 0xd7);
    ld16(reg16(REG_HL), 0x0a08);
    ld16(reg16(REG_BC), 0xf5f7);

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
    if(~BIT(A, 3) && BIT(n, 3)) 
        F |= FLAG_H;
    if(~BIT(A_old, 7) && BIT(n, 7))
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
    F |= BIT(r, b) ? 0x0 : FLAG_Z;
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
