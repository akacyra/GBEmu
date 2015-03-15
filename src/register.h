#ifndef GUARD_REGISTER_H
#define GUARD_REGISTER_H

#include <stdint.h>
#include <iostream>
#include <iomanip>

//
// A pair of 8 bit registers that also form a 16 bit register.
//

class Register
{
    public:
        Register() : h(0), l(0), hl(0) { };

        uint8_t high() const { return h; };
        uint8_t low() const { return l; };
        uint16_t pair() const { return hl; };

        void operator=(const uint16_t &rhs) {
            hl = rhs;
            h = (rhs & 0xff00) >> 8;
            l = rhs & 0x00ff;
        };
        void set_high(uint8_t val) {
            h = val;
            hl = (hl & 0x00ff) | (val << 8);
        };
        void set_low(uint8_t val) {
            l = val;
            hl = (hl & 0xff00) | val;
        };

        friend std::ostream& operator<<(std::ostream& os, 
                const Register &r) {
            os << std::hex << (int)r.h 
               << " : " << (int)r.l << std::dec;
            return os;
        };

        Register operator++() { *this = hl + 1; return *this; };
        Register operator++(int) { *this = hl + 1; return *this; };
        Register operator--() { *this = hl - 1; return *this; };
        Register operator--(int) { *this = hl - 1; return *this; };

        operator uint16_t() const { return hl; };

    private:
        uint8_t h, l;
        uint16_t hl;
};

#endif
