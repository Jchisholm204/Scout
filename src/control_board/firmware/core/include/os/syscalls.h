/**
 * @file syscalls.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-12
 * @modified Last Modified: 2026-01-12
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _SYSCALLS_H_
#define _SYSCALLS_H_
#include "drivers/serial.h"

#include <sys/stat.h>

/**
 * @brief Register the STDIO Serial Port
 *
 * @param pSerial Serial Port to use as STDIO
 * @return Serial Error code
 */
extern int register_stdio(Serial_t *pSerial);

extern int _fstat(int fd, struct stat *st);

extern void *_sbrk(ptrdiff_t incr);

extern int _open(const char *path);

extern int _close(int fd);

extern int _isatty(int fd);

extern int _lseek(int fd, int ptr, int dir);

extern void _exit(int status);

extern void _kill(int pid, int sig);

extern int _getpid(void);

extern int _write(int fd, char *ptr, int len);

extern int _read(int fd, char *ptr, int len);

extern int _link(const char *a, const char *b);

extern int _unlink(const char *a);

extern int _stat(const char *path, struct stat *st);

extern int mkdir(const char *path, mode_t mode);

#endif
