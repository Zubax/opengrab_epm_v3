/*
 * OpenGrab EPM - Electropermanent Magnet
 * Copyright (C) 2015  Zubax Robotics <info@zubax.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Pavel Kirienko <pavel.kirienko@zubax.com>
 *
 * Standard library stubs.
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
