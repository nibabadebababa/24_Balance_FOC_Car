//
// Created by Administrator on 2024/3/28.
// 因为clion使用的编译器和keil使用的编译器不同,
// 所以在clion中无法使用keil的库函数, 例如printf, scanf等, 为了解决这个问题,
// 我们可以使用retarget技术, 通过串口将数据发送到电脑上,
// 这样就可以在电脑上看到数据了 所以在keil中不需要用到这个文件,
// 在clion中需要用到这个文件
//
#ifndef _RETARGET_H__
#define _RETARGET_H__

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <sys/stat.h>

void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char *ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char *ptr, int len);
int _fstat(int fd, struct stat *st);

#endif // #ifndef _RETARGET_H__