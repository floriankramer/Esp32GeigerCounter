#include "queue.h"

void fsq_init(FixedSizeQueue_t *queue, uint16_t entry_size, uint8_t max_entries) {
  queue->entry_size = entry_size;
  queue->capacity = max_entries;
  queue->pos = 0;
  queue->length = 0;
  queue->data = malloc(max_entries * entry_size);
  queue->lengths = malloc(max_entries);
}

void fsq_delete(FixedSizeQueue_t *queue) {
  free(queue->data);
  free(queue->lengths);
  queue->data = 0;
  queue->length = 0;
  queue->capacity = 0;
  queue->pos = 0;
}

void fsq_front(FixedSizeQueue_t *queue, uint8_t **data, uint8_t *length) {
  if (queue->length > 0) {
    *data = queue->data + (queue->pos * queue->entry_size);
    *length = queue->lengths[queue->pos]; 
  } else {
    *data = 0;
    *length = 0;
  }
}

uint8_t fsq_size(FixedSizeQueue_t *queue) {
  return queue->length;
}

uint8_t fsq_capacity(FixedSizeQueue_t *queue) {
  return queue->capacity;
}

void fsq_pop(FixedSizeQueue_t *queue) {
  if (queue->length > 0) {
    ++queue->pos;
    queue->pos %= queue->capacity;
    --queue->length;
  }
}

void fsq_clear(FixedSizeQueue_t *queue) {
  queue->pos = 0;
  queue->length = 0;
}

void fsq_push(FixedSizeQueue_t *queue, const uint8_t *src, uint8_t count) {
  if (count > queue->entry_size) {
    count = queue->entry_size;
  }
  uint8_t t = queue->pos + queue->length;
  t %= queue->capacity;
  if (queue->length < queue->capacity) {
    // There still is a free slot
    ++queue->length;
  } else {
    // The queue is already full, move the queues head forward by one 
    queue->pos++;
    queue->pos %= queue->capacity;
  }
  memcpy(queue->data + (t * queue->entry_size), src, count);
  queue->lengths[t] = count;
}

void fsq_push_ref(FixedSizeQueue_t *queue, uint8_t **data, uint8_t **length) {
  uint8_t t = queue->pos + queue->length;
  t %= queue->capacity;
  if (queue->length < queue->capacity) {
    // There still is a free slot
    ++queue->length;
  } else {
    // The queue is already full, move the queues head forward by one 
    queue->pos++;
    queue->pos %= queue->capacity;
  }
  *data = queue->data + (t * queue->entry_size);
  *length = queue->lengths + t;
}

char fsq_push_save(FixedSizeQueue_t *queue, const uint8_t *src, uint8_t count) {
 if (queue->length >= queue->capacity) {
   return 0;
 }
 fsq_push(queue, src, count);
 return 1;
}
