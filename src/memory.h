#ifndef GUARD_MEMORY_H
#define GUARD_MEMORY_H

#include <iostream> 
#include <vector>
#include <stdint.h>

//
// Variable-length byte addressable memory.
//

class Memory
{
    public:
        typedef std::vector<uint8_t>::size_type usize;

        Memory(usize size);

        uint8_t& operator[](usize addr) { return mem[addr]; };
        const uint8_t& operator[](usize addr) const { return mem[addr]; };

        uint16_t get16(usize addr) const { 
            return ((uint16_t)(mem[addr+1]) << 8) + mem[addr];
        };
        void set16(usize addr, uint16_t val) {
            mem[addr] = val & 0x00ff;
            mem[addr+1] = (val & 0xff00) >> 8;
        };

        void load_bytes_to(usize addr, std::vector<uint8_t> bytes);

        void print(std::ostream& os, usize addr) const;
        void print(std::ostream& os, usize begin, usize end) const;
        

    private:
        std::vector<uint8_t> mem;
};

#endif
