#pragma once

/*
 * Ring buffer implementation helpers
 */

/* Wrap up buffer index */
static inline unsigned ring_wrap(unsigned size, unsigned idx)
{
	return idx >= size ? idx - size : idx;
}

/* Returns the number of bytes available in buffer */
static inline unsigned ring_data_avail(unsigned size, unsigned head, unsigned tail)
{
	if (head >= tail)
		return head - tail;
	else
		return size + head - tail;
}

/* Returns the amount of free space available in buffer */
static inline unsigned ring_space_avail(unsigned size, unsigned head, unsigned tail)
{
	return size - ring_data_avail(size, head, tail) - 1;
}

/* Returns the number of contiguous data bytes available in buffer */
static inline unsigned ring_data_contig(unsigned size, unsigned head, unsigned tail)
{
	if (head >= tail)
		return head - tail;
	else
		return size - tail;
}

/* Returns the amount of contiguous space available in buffer */
static inline unsigned ring_space_contig(unsigned size, unsigned head, unsigned tail)
{
	if (head >= tail)
		return (tail ? size : size - 1) - head;
	else
		return tail - head - 1;
}

/* Returns the amount of free space available after wrapping up the head */
static inline unsigned ring_space_wrapped(unsigned size, unsigned head, unsigned tail)
{
	if (head < tail || !tail)
		return 0;
	else
		return tail - 1;
}

