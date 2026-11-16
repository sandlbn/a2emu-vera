/* hello.c -- booted straight from disk by boot.s; main never returns */
#include <conio.h>

int main(void)
{
    clrscr();
    cputs("Hello, World!\r\n");
    cputs("cc65 + boot loader, no DOS\r\n");
    for (;;)
        ;
}
