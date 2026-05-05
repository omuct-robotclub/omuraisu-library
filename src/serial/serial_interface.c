#include "serial/serial_interface.h"

bool serial_port_open(SerialPort* port) {
  if (port == 0 || port->open == 0) {
    return false;
  }
  return port->open(port->impl);
}

void serial_port_close(SerialPort* port) {
  if (port == 0 || port->close == 0) {
    return;
  }
  port->close(port->impl);
}

bool serial_port_write(SerialPort* port, const SerialMessage* msg) {
  if (port == 0 || port->write == 0) {
    return false;
  }
  return port->write(port->impl, msg);
}

bool serial_port_read(SerialPort* port, SerialMessage* msg) {
  if (port == 0 || port->read == 0) {
    return false;
  }
  return port->read(port->impl, msg);
}

void serial_port_start_read(SerialPort* port) {
  if (port == 0 || port->start_read == 0) {
    return;
  }
  port->start_read(port->impl);
}

void serial_port_stop_read(SerialPort* port) {
  if (port == 0 || port->stop_read == 0) {
    return;
  }
  port->stop_read(port->impl);
}

void serial_port_set_rx_callback(SerialPort* port, SerialRxCallback callback,
                                 void* user_arg) {
  if (port == 0 || port->set_rx_callback == 0) {
    return;
  }
  port->set_rx_callback(port->impl, callback, user_arg);
}

void serial_port_destroy(SerialPort* port) {
  if (port == 0 || port->destroy == 0) {
    return;
  }
  port->destroy(port->impl);
}
