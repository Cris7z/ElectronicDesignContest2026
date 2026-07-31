#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "stm32f10x.h"

extern char end;
static char *s_heap_end;

int _write(int file, char *data, int length)
{
    int index;
    (void)file;
    for(index = 0; index < length; index++)
    {
        while((USART1->SR & USART_SR_TXE) == 0U)
        {
        }
        USART1->DR = (u16)(u8)data[index];
    }
    return length;
}

int _read(int file, char *data, int length)
{
    (void)file;
    (void)data;
    (void)length;
    errno = EAGAIN;
    return -1;
}

int _close(int file)
{
    (void)file;
    errno = ENOSYS;
    return -1;
}

int _fstat(int file, struct stat *status)
{
    (void)file;
    status->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _lseek(int file, int pointer, int direction)
{
    (void)file;
    (void)pointer;
    (void)direction;
    errno = ENOSYS;
    return -1;
}

int _kill(int process, int signal)
{
    (void)process;
    (void)signal;
    errno = EINVAL;
    return -1;
}

int _getpid(void)
{
    return 1;
}

void *_sbrk(ptrdiff_t increment)
{
    char *previous;
    register char *stack_pointer __asm("sp");

    if(s_heap_end == 0)
        s_heap_end = &end;
    previous = s_heap_end;
    if((s_heap_end + increment) > stack_pointer)
    {
        errno = ENOMEM;
        return (void *)-1;
    }
    s_heap_end += increment;
    return previous;
}
