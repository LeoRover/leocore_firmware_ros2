#pragma once

#include <cstdint>
#include <cstdlib>

uint32_t max_used_heap();
void free_all_heap();
void set_heap(uint32_t pointer);

void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
                             void* state);
