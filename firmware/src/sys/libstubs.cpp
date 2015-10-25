/*
 * Copyright (c) 2015 Zubax Robotics, zubax.com
 * Distributed under the MIT License, available in the file LICENSE.
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 *
 * Standard library stubs
 */

#include <cstdlib>
#include <unistd.h>
#include <sys/types.h>

#if __GNUC__
# pragma GCC diagnostic ignored "-Wmissing-declarations"
#endif

void* __dso_handle;

void* operator new(size_t)
{
    std::abort();
    return reinterpret_cast<void*>(0xFFFFFFFF);
}

void* operator new[](size_t)
{
    std::abort();
    return reinterpret_cast<void*>(0xFFFFFFFF);
}

void operator delete(void*)
{
    std::abort();
}

void operator delete[](void*)
{
    std::abort();
}

namespace __gnu_cxx
{

void __verbose_terminate_handler()
{
    std::abort();
}

}

/*
 * libstdc++ stubs
 */
extern "C"
{

int __aeabi_atexit(void*, void(*)(void*), void*)
{
    return 0;
}

__extension__ typedef int __guard __attribute__((mode (__DI__)));

void __cxa_atexit(void(*)(void *), void*, void*)
{
}

int __cxa_guard_acquire(__guard* g)
{
    return !*g;
}

void __cxa_guard_release (__guard* g)
{
    *g = 1;
}

void __cxa_guard_abort (__guard*)
{
}

void __cxa_pure_virtual()
{
    std::abort();
}

}

/**
 * This function re-defines the standard ::rand(), which is used by the class uavcan::DynamicNodeIDClient.
 * Redefinition is normally not needed, but GCC 4.9 tends to generate broken binaries if it is not redefined.
 */
extern "C"
int rand()
{
    static int x = 1;
    x = x * 48271 % 2147483647;
    return x;
}

/*
 * stdio
 */
extern "C"
{

__attribute__((used))
void abort()
{
    while (true) { }
}

int _read_r(struct _reent*, int, char*, int)
{
    return -1;
}

int _lseek_r(struct _reent*, int, int, int)
{
    return -1;
}

int _write_r(struct _reent*, int, char*, int)
{
    return -1;
}

int _close_r(struct _reent*, int)
{
    return -1;
}

__attribute__((used))
caddr_t _sbrk_r(struct _reent*, int)
{
    return 0;
}

int _fstat_r(struct _reent*, int, struct stat*)
{
    return -1;
}

int _isatty_r(struct _reent*, int)
{
    return -1;
}

void _exit(int)
{
    abort();
}

pid_t _getpid(void)
{
    return 1;
}

void _kill(pid_t)
{
}

}
