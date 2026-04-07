// Copyright 2025 Lejurobot. All rights reserved.

#pragma once

#include <cstdio>

#define HW_TAG "HW"
#define HWLOG_I(format, ...)    printf("[%s][INFO] " format "\n", HW_TAG, ##__VA_ARGS__)
#define HWLOG_D(format, ...)    printf("[%s][DEBUG] " format "\n", HW_TAG, ##__VA_ARGS__)
#define HWLOG_E(format, ...)    fprintf(stderr, "[%s][ERROR] " format "\n", HW_TAG, ##__VA_ARGS__)
#define HWLOG_T(format, ...)    printf("[%s][TRACE] " format "\n", HW_TAG, ##__VA_ARGS__)
#define HWLOG_W(format, ...)    printf("[%s][WARN] " format "\n", HW_TAG, ##__VA_ARGS__)
#define HWLOG_SUCCESS(format, ...)  printf("\033[32m[%s][OK] " format "\033[0m\n", HW_TAG, ##__VA_ARGS__)
#define HWLOG_WARNING(format, ...)  printf("\033[33m[%s][WARN] " format "\033[0m\n", HW_TAG, ##__VA_ARGS__)
#define HWLOG_FAILURE(format, ...)  fprintf(stderr, "\033[31m[%s][ERROR] " format "\033[0m\n", HW_TAG, ##__VA_ARGS__)
