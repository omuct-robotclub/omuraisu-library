#ifndef SERIAL_CUBE_H
#define SERIAL_CUBE_H

#include <stdbool.h>
#include <stdint.h>

#include "serial/serial_interface.h"

#ifndef SERIAL_CUBE_RX_QUEUE_SIZE
#define SERIAL_CUBE_RX_QUEUE_SIZE 16
#endif

typedef struct {
  bool (*open)(void* hal_context);
  void (*close)(void* hal_context);
  bool (*write)(void* hal_context, const SerialMessage* msg);
  bool (*read_hw)(void* hal_context, SerialMessage* msg);
  void (*start_read)(void* hal_context);
  void (*stop_read)(void* hal_context);
} SerialCubeOps;

typedef struct {
  SerialPort port;

  void* hal_context;
  SerialCubeOps ops;

  SerialMessage rx_queue[SERIAL_CUBE_RX_QUEUE_SIZE];
  uint8_t rx_head;
  uint8_t rx_tail;
  uint8_t rx_count;

  uint32_t rx_overflow_count;

  SerialRxCallback rx_callback;
  void* rx_callback_user_arg;
} SerialCube;

void serial_cube_init(SerialCube* cube, void* hal_context,
                      const SerialCubeOps* ops);

SerialPort* serial_cube_port(SerialCube* cube);

void serial_cube_set_rx_callback(SerialCube* cube, SerialRxCallback callback,
                                 void* user_arg);

bool serial_cube_poll(SerialCube* cube, SerialMessage* msg);

uint32_t serial_cube_get_rx_overflow_count(const SerialCube* cube);

bool serial_cube_open(SerialCube* cube);

void serial_cube_close(SerialCube* cube);

void serial_cube_start_read(SerialCube* cube);

void serial_cube_stop_read(SerialCube* cube);

void serial_cube_on_rx_pending(SerialCube* cube);

#endif  // SERIAL_CUBE_H
