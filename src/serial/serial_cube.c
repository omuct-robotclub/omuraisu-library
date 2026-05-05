#include "serial/serial_cube.h"

#include <string.h>

static bool serial_cube_queue_push(SerialCube* cube, const SerialMessage* msg) {
  if (cube->rx_count >= SERIAL_CUBE_RX_QUEUE_SIZE) {
    cube->rx_overflow_count++;
    return false;
  }

  cube->rx_queue[cube->rx_head] = *msg;
  cube->rx_head = (uint8_t)((cube->rx_head + 1U) % SERIAL_CUBE_RX_QUEUE_SIZE);
  cube->rx_count++;
  return true;
}

static bool serial_cube_queue_pop(SerialCube* cube, SerialMessage* msg) {
  if (cube->rx_count == 0U) {
    return false;
  }

  *msg = cube->rx_queue[cube->rx_tail];
  cube->rx_tail = (uint8_t)((cube->rx_tail + 1U) % SERIAL_CUBE_RX_QUEUE_SIZE);
  cube->rx_count--;
  return true;
}

static bool serial_cube_port_open_impl(void* self) {
  SerialCube* cube = (SerialCube*)self;
  if (cube->ops.open == 0) {
    return false;
  }
  return cube->ops.open(cube->hal_context);
}

static void serial_cube_port_close_impl(void* self) {
  SerialCube* cube = (SerialCube*)self;
  if (cube->ops.close == 0) {
    return;
  }
  cube->ops.close(cube->hal_context);
}

static bool serial_cube_port_write_impl(void* self, const SerialMessage* msg) {
  SerialCube* cube = (SerialCube*)self;
  if (cube->ops.write == 0) {
    return false;
  }
  return cube->ops.write(cube->hal_context, msg);
}

static bool serial_cube_port_read_impl(void* self, SerialMessage* msg) {
  SerialCube* cube = (SerialCube*)self;
  return serial_cube_queue_pop(cube, msg);
}

static void serial_cube_port_start_read_impl(void* self) {
  SerialCube* cube = (SerialCube*)self;
  if (cube->ops.start_read == 0) {
    return;
  }
  cube->ops.start_read(cube->hal_context);
}

static void serial_cube_port_stop_read_impl(void* self) {
  SerialCube* cube = (SerialCube*)self;
  if (cube->ops.stop_read == 0) {
    return;
  }
  cube->ops.stop_read(cube->hal_context);
}

static void serial_cube_port_set_rx_callback_impl(void* self,
                                                  SerialRxCallback callback,
                                                  void* user_arg) {
  SerialCube* cube = (SerialCube*)self;
  serial_cube_set_rx_callback(cube, callback, user_arg);
}

static void serial_cube_port_destroy_impl(void* self) { (void)self; }

void serial_cube_init(SerialCube* cube, void* hal_context,
                      const SerialCubeOps* ops) {
  memset(cube, 0, sizeof(*cube));

  cube->hal_context = hal_context;
  if (ops != 0) {
    cube->ops = *ops;
  }

  cube->port.open = serial_cube_port_open_impl;
  cube->port.close = serial_cube_port_close_impl;
  cube->port.write = serial_cube_port_write_impl;
  cube->port.read = serial_cube_port_read_impl;
  cube->port.start_read = serial_cube_port_start_read_impl;
  cube->port.stop_read = serial_cube_port_stop_read_impl;
  cube->port.set_rx_callback = serial_cube_port_set_rx_callback_impl;
  cube->port.destroy = serial_cube_port_destroy_impl;
  cube->port.impl = cube;
}

SerialPort* serial_cube_port(SerialCube* cube) { return &cube->port; }

void serial_cube_set_rx_callback(SerialCube* cube, SerialRxCallback callback,
                                 void* user_arg) {
  cube->rx_callback = callback;
  cube->rx_callback_user_arg = user_arg;
}

bool serial_cube_poll(SerialCube* cube, SerialMessage* msg) {
  return serial_cube_queue_pop(cube, msg);
}

uint32_t serial_cube_get_rx_overflow_count(const SerialCube* cube) {
  return cube->rx_overflow_count;
}

bool serial_cube_open(SerialCube* cube) {
  if (cube->ops.open == 0) {
    return false;
  }
  return cube->ops.open(cube->hal_context);
}

void serial_cube_close(SerialCube* cube) {
  if (cube->ops.close == 0) {
    return;
  }
  cube->ops.close(cube->hal_context);
}

void serial_cube_start_read(SerialCube* cube) {
  if (cube->ops.start_read == 0) {
    return;
  }
  cube->ops.start_read(cube->hal_context);
}

void serial_cube_stop_read(SerialCube* cube) {
  if (cube->ops.stop_read == 0) {
    return;
  }
  cube->ops.stop_read(cube->hal_context);
}

void serial_cube_on_rx_pending(SerialCube* cube) {
  SerialMessage msg;

  if (cube->ops.read_hw == 0) {
    return;
  }

  while (cube->ops.read_hw(cube->hal_context, &msg)) {
    serial_cube_queue_push(cube, &msg);
    if (cube->rx_callback != 0) {
      cube->rx_callback(&msg, cube->rx_callback_user_arg);
    }
  }
}
