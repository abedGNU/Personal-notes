// hello.c

// gcc hello.c -o hello
// gcc -o hello hello.c

#include <stdio.h>
void main(void)
{
    printf("hello world!\n");
}

/*
Genearate hello.o
    gcc -c hello.c

Generate output file hello (linux executable)
    gcc -o hello hello.c

strings command look at text strings
    strings hello

The utility "file" can be used to display the type of object files and executable files:
    file hello.c
    file hello.o
    file hello

The utility nm lists symbol table of object files:
    nm hello.o
    nm hello | grep main

ldd Utility - List Dynamic-Link Libraries
    ldd hello

objdump -d hello > disassembly.asm
objdump -M intel -d hello > disassembly.asm

ltrace hello
strace hello

*/