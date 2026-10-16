/* Test shim for e2e_operator.py: the operator asserts DTR and RTS with TIOCMGET and
   TIOCMSET, which a pseudo-terminal does not implement (ENOTTY), so it would abort at
   start. Preloaded into the operator, this makes those two ioctls succeed on a pty and
   touches nothing else. Build: make -C test/host build/ptyshim.so */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <stdarg.h>
#include <stddef.h>
#include <sys/ioctl.h>
#include <termios.h>

int ioctl(int fd, unsigned long req, ...)
{
    va_list ap;
    va_start(ap, req);
    void *arg = va_arg(ap, void *);
    va_end(ap);
    static int (*real)(int, unsigned long, ...) = NULL;
    if (!real) {
        real = (int (*)(int, unsigned long, ...))dlsym(RTLD_NEXT, "ioctl");
    }
    const int r = real(fd, req, arg);
    if (r < 0 && errno == ENOTTY && (req == TIOCMGET || req == TIOCMSET)) {
        if (req == TIOCMGET) {
            *(int *)arg = TIOCM_DTR | TIOCM_RTS;
        }
        return 0;
    }
    return r;
}
