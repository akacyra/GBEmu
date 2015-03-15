#include "memory.h"
#include <algorithm>

using std::vector;

Memory::Memory(usize size) : mem(std::vector<uint8_t>(size)) { };

void Memory::print(std::ostream& os, usize addr) const
{
    os << std::hex << "0x" << (int)addr << " = 0x" 
       << (int)mem[addr] << std::dec << std::endl;
} // print()

void Memory::print(std::ostream& os, usize begin, usize end) const
{
    os << std::hex;
    auto it = mem.begin() + begin, end_it = mem.begin() + end;
    for(usize addr = begin; it != end_it; it++, addr++)
        os << "0x" << (int)addr << " = 0x" << (int)*it << std::endl;
    os << std::dec;
} // print() 

void Memory::load_bytes_to(usize addr, vector<uint8_t> bytes)
{
    copy(bytes.begin(), bytes.end(), mem.begin() + addr);
} // load_bytes_at()
