#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SERIAL_MESSAGE_MAX_LEN
#define SERIAL_MESSAGE_MAX_LEN 64
#endif

/// @brief シリアル通信用の受信/送信メッセージ
typedef struct {
  uint8_t data[SERIAL_MESSAGE_MAX_LEN];
  uint16_t len;
} SerialMessage;

typedef void (*SerialRxCallback)(const SerialMessage* msg, void* user_arg);

/// @brief プラットフォーム非依存のシリアル抽象インターフェース
typedef struct {
  bool (*open)(void* self);
  void (*close)(void* self);
  bool (*write)(void* self, const SerialMessage* msg);
  bool (*read)(void* self, SerialMessage* msg);
  void (*start_read)(void* self);
  void (*stop_read)(void* self);
  void (*set_rx_callback)(void* self, SerialRxCallback callback,
                          void* user_arg);
  void (*destroy)(void* self);

  void* impl;
} SerialPort;

bool serial_port_open(SerialPort* port);

void serial_port_close(SerialPort* port);

bool serial_port_write(SerialPort* port, const SerialMessage* msg);

bool serial_port_read(SerialPort* port, SerialMessage* msg);

void serial_port_start_read(SerialPort* port);

void serial_port_stop_read(SerialPort* port);

void serial_port_set_rx_callback(SerialPort* port, SerialRxCallback callback,
                                 void* user_arg);

void serial_port_destroy(SerialPort* port);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SERIAL_INTERFACE_H
