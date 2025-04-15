#include <cstring>

#include "app/configuration.hpp"
#include "app/microros_allocators.hpp"

static uint8_t heap[UROS_HEAP_SIZE];
static size_t current_pointer = 0;

uint32_t heap_get_current_pointer() {
  return current_pointer;
}

void heap_free_all() {
  current_pointer = 0;
}

void heap_set_current_pointer(uint32_t pointer) {
  current_pointer = pointer;
}

void assert_position() {
  if (current_pointer >= sizeof(heap)) {
    while (1) {
    }
  }
}

void *microros_allocate(size_t size, void *state) {
  if (size % 4 != 0) {
    size += 4 - (size % 4);
  }

  size_t p = current_pointer;

  current_pointer += size;

  assert_position();
  return (void *)&heap[p];
}

void microros_deallocate(void *pointer, void *state) {
  (void)state;
  (void)pointer;
}

void *microros_reallocate(void *pointer, size_t size, void *state) {
  if (size % 4 != 0) {
    size += 4 - (size % 4);
  }

  size_t p = current_pointer;

  current_pointer += size;

  // Careful! pointer have less than size memory, garbage is gonna be copied!
  std::memcpy(&heap[p], pointer, size);

  assert_position();
  return (void *)&heap[p];
}

void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void *state) {
  size_t size = number_of_elements * size_of_element;

  if (size % 4 != 0) {
    size += 4 - (size % 4);
  }

  size_t p = current_pointer;

  current_pointer += size;

  std::memset(&heap[p], 0, size);

  assert_position();
  return (void *)&heap[p];
}
