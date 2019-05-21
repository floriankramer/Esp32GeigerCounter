#ifndef QUEUE_QUEUE_H_
#define QUEUE_QUEUE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct FixedSizeQueue {
  uint16_t entry_size;
  uint8_t *data;
  uint8_t *lengths;
  uint8_t pos;
  uint8_t length;
  uint8_t capacity;
};

typedef struct FixedSizeQueue FixedSizeQueue_t;

// Initializes a new queue
void fsq_init(FixedSizeQueue_t *queue, uint16_t entry_size,
              uint8_t max_entries);

// Frees the queues memory
void fsq_delete(FixedSizeQueue_t *queue);

// Retrieves the element at the front of the queue. Data is set to NULL and
// length to 0 if the queue is empty
void fsq_front(FixedSizeQueue_t *queue, uint8_t **data, uint8_t *length);

// Returns the size of the queue
uint8_t fsq_size(FixedSizeQueue_t *queue);

uint8_t fsq_capacity(FixedSizeQueue_t *queue);

// Removes the first element from the queue. Does nothin if the queue is empty
void fsq_pop(FixedSizeQueue_t *queue);

// Removes all elements from the queue
void fsq_clear(FixedSizeQueue_t *queue);

// Copies count many bytes from src into the element behind the end of the
// queue. Never copies more than the queues entry size many bytes. If the queue
// is full the first element is replaced with the new data and becomes the new
// last element.
void fsq_push(FixedSizeQueue_t *queue, const uint8_t *src, uint8_t count);

// Adds a new element onto the queue and stores the location at which its
// data and length are stored in the data and length pointers. It is the
// user's responsibility to ensure that no more than the queues entry size
// bytes are written to data.
void fsq_push_ref(FixedSizeQueue_t *queue, uint8_t **data, uint8_t **length);

// Copies count many bytes from src into the element behind the end of the
// queue. Never copies more than the queues entry size many bytes. If the queue
// is full no data is copied and 0 is returned. Otherwise returns 1.
char fsq_push_save(FixedSizeQueue_t *queue, const uint8_t *src, uint8_t count);

#ifdef __cplusplus
}
#endif
#endif
