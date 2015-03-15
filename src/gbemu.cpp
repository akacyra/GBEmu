#include "cpu.h"

int main(int argc, char *argv[])
{
    CPU z80;

    z80.run();

    z80.print_registers();

    return 0;
} // main()
