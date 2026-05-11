#include <zephyr/kernel.h>
#include <app/lib/system/block_heap.h>

K_HEAP_DEFINE(block_heap, CONFIG_SYNCHROFLY_BLOCK_HEAP_SIZE);

void *block_malloc(size_t size)
{
	return k_heap_alloc(&block_heap, size, K_NO_WAIT);
}

void block_free(void *ptr)
{
	k_heap_free(&block_heap, ptr);
}

struct k_heap *block_heap_get(void)
{
	return &block_heap;
}
