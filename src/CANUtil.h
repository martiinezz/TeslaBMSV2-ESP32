#pragma once

#include <Arduino.h>

typedef struct CAN_message_t {
    uint32_t id;          
    struct {
        uint8_t extended:1; 
    } flags;
    uint8_t len;          
    uint8_t buf[8];
} CAN_message_t;

typedef struct CAN_filter_t {
    uint32_t id;
    struct {
        uint8_t extended:1;
    } flags;
} CAN_filter_t;