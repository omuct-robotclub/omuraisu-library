#ifndef SERIAL_STM32_H
#define SERIAL_STM32_H

#include <stdbool.h>
#include <stdint.h>

#include "serial/serial_cube.h"

typedef struct {
  SerialCube* cube;
  void* handle;

  uint8_t rx_byte;
  uint8_t rx_buffer[SERIAL_MESSAGE_MAX_LEN * 2U];
  uint16_t rx_head;
  uint16_t rx_tail;
  uint16_t rx_count;
} SerialStm32Context;

void serial_stm32_context_init(SerialStm32Context* context, SerialCube* cube,
                               void* handle);

void serial_stm32_make_ops(SerialCubeOps* ops);

bool serial_stm32_register(SerialStm32Context* context);

void serial_stm32_unregister(SerialStm32Context* context);

void serial_stm32_dispatch_rx(void* handle);

#endif  // SERIAL_STM32_H
