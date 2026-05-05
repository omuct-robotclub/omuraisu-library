#include "serial/serial_stm32.h"

#include <string.h>

#ifndef OMURAISU_SERIAL_STM32_MAX_CONTEXTS
#define OMURAISU_SERIAL_STM32_MAX_CONTEXTS 4
#endif

#ifndef OMURAISU_SERIAL_STM32_HAL_HEADER
#define OMURAISU_SERIAL_STM32_HAL_HEADER "main.h"
#endif

#ifndef OMURAISU_SERIAL_STM32_TX_TIMEOUT_MS
#define OMURAISU_SERIAL_STM32_TX_TIMEOUT_MS 10U
#endif

#ifdef OMURAISU_SERIAL_STM32_ENABLE
#include OMURAISU_SERIAL_STM32_HAL_HEADER

typedef struct {
  SerialStm32Context* context;
} SerialStm32Slot;

static SerialStm32Slot g_slots[OMURAISU_SERIAL_STM32_MAX_CONTEXTS];

static SerialStm32Context* serial_stm32_find_context(void* handle) {
  for (int i = 0; i < OMURAISU_SERIAL_STM32_MAX_CONTEXTS; ++i) {
    if (g_slots[i].context != 0 && g_slots[i].context->handle == handle) {
      return g_slots[i].context;
    }
  }
  return 0;
}

static bool serial_stm32_rx_push(SerialStm32Context* context, uint8_t value) {
  uint16_t buffer_size = (uint16_t)(sizeof(context->rx_buffer));
  if (context->rx_count >= buffer_size) {
    return false;
  }

  context->rx_buffer[context->rx_head] = value;
  context->rx_head = (uint16_t)((context->rx_head + 1U) % buffer_size);
  context->rx_count++;
  return true;
}

static bool serial_stm32_rx_pop(SerialStm32Context* context, uint8_t* value) {
  uint16_t buffer_size = (uint16_t)(sizeof(context->rx_buffer));
  if (context->rx_count == 0U) {
    return false;
  }

  *value = context->rx_buffer[context->rx_tail];
  context->rx_tail = (uint16_t)((context->rx_tail + 1U) % buffer_size);
  context->rx_count--;
  return true;
}

static bool serial_stm32_open(void* self) {
  (void)self;
  return true;
}

static void serial_stm32_close(void* self) {
  SerialStm32Context* context = (SerialStm32Context*)self;
  UART_HandleTypeDef* huart = (UART_HandleTypeDef*)context->handle;
  (void)HAL_UART_AbortReceive_IT(huart);
}

static bool serial_stm32_write(void* self, const SerialMessage* msg) {
  SerialStm32Context* context = (SerialStm32Context*)self;
  UART_HandleTypeDef* huart = (UART_HandleTypeDef*)context->handle;

  if (msg == 0 || msg->len == 0) {
    return false;
  }

  return HAL_UART_Transmit(huart, (uint8_t*)msg->data, msg->len,
                           OMURAISU_SERIAL_STM32_TX_TIMEOUT_MS) == HAL_OK;
}

static bool serial_stm32_read_hw(void* self, SerialMessage* msg) {
  SerialStm32Context* context = (SerialStm32Context*)self;
  uint16_t count = 0;

  if (msg == 0) {
    return false;
  }

  while (count < SERIAL_MESSAGE_MAX_LEN) {
    uint8_t value = 0;
    if (!serial_stm32_rx_pop(context, &value)) {
      break;
    }
    msg->data[count++] = value;
  }

  msg->len = count;
  return count > 0;
}

static void serial_stm32_start_read(void* self) {
  SerialStm32Context* context = (SerialStm32Context*)self;
  UART_HandleTypeDef* huart = (UART_HandleTypeDef*)context->handle;
  (void)HAL_UART_Receive_IT(huart, &context->rx_byte, 1U);
}

static void serial_stm32_stop_read(void* self) {
  SerialStm32Context* context = (SerialStm32Context*)self;
  UART_HandleTypeDef* huart = (UART_HandleTypeDef*)context->handle;
  (void)HAL_UART_AbortReceive_IT(huart);
}

void serial_stm32_context_init(SerialStm32Context* context, SerialCube* cube,
                               void* handle) {
  memset(context, 0, sizeof(*context));
  context->cube = cube;
  context->handle = handle;
}

void serial_stm32_make_ops(SerialCubeOps* ops) {
  ops->open = serial_stm32_open;
  ops->close = serial_stm32_close;
  ops->write = serial_stm32_write;
  ops->read_hw = serial_stm32_read_hw;
  ops->start_read = serial_stm32_start_read;
  ops->stop_read = serial_stm32_stop_read;
}

bool serial_stm32_register(SerialStm32Context* context) {
  for (int i = 0; i < OMURAISU_SERIAL_STM32_MAX_CONTEXTS; ++i) {
    if (g_slots[i].context == context) {
      return true;
    }
    if (g_slots[i].context == 0) {
      g_slots[i].context = context;
      return true;
    }
  }
  return false;
}

void serial_stm32_unregister(SerialStm32Context* context) {
  for (int i = 0; i < OMURAISU_SERIAL_STM32_MAX_CONTEXTS; ++i) {
    if (g_slots[i].context == context) {
      g_slots[i].context = 0;
      return;
    }
  }
}

void serial_stm32_dispatch_rx(void* handle) {
  SerialStm32Context* context = serial_stm32_find_context(handle);
  if (context == 0 || context->cube == 0) {
    return;
  }

  serial_stm32_rx_push(context, context->rx_byte);
  (void)HAL_UART_Receive_IT((UART_HandleTypeDef*)context->handle,
                            &context->rx_byte, 1U);
  serial_cube_on_rx_pending(context->cube);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  serial_stm32_dispatch_rx(huart);
}

#else

void serial_stm32_context_init(SerialStm32Context* context, SerialCube* cube,
                               void* handle) {
  (void)context;
  (void)cube;
  (void)handle;
}

void serial_stm32_make_ops(SerialCubeOps* ops) { memset(ops, 0, sizeof(*ops)); }

bool serial_stm32_register(SerialStm32Context* context) {
  (void)context;
  return false;
}

void serial_stm32_unregister(SerialStm32Context* context) { (void)context; }

void serial_stm32_dispatch_rx(void* handle) { (void)handle; }

#endif  // OMURAISU_SERIAL_STM32_ENABLE
