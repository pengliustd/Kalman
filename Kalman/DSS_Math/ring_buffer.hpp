
/**
 * @file ringbuffer_.h
 */

#include <cstdint>
#include <cstdio>
#include <string>


template <typename T>
class RingBuffer
{
public:
	RingBuffer()
	{
		buffer_ = NULL;
		head_ = tail_ = size_ = 0;
		firstWrite_ = true;
	}
	~RingBuffer() { delete[] buffer_; }

	bool Allocate(uint64_t size)
	{
		if (size <= 0) {
			return false;
		}

		if (buffer_ != NULL) {
			delete[] buffer_;
		}

		buffer_ = new T[size];

		if (buffer_ == NULL) {
			return false;
		}

		size_ = size;
		// set the time elements to zero so that bad data is not retrieved from the buffers
		for (uint64_t index = 0; index < size_; index++) {
			buffer_[index].time_us = 0;
		}
		firstWrite_ = true;
		return true;
	}

	void Unallocate()
	{
		if (buffer_ != NULL) {
			delete[] buffer_;
		}
	}

	inline void Push(T sample)
	{
		uint64_t head_new = head_;

		if (firstWrite_) {
			head_new = head_;

		}
		else {
			head_new = (head_ + 1) % size_;
		}

		buffer_[head_new] = sample;
		head_ = head_new;

		// move tail if we overwrite it
		if (head_ == tail_ && !firstWrite_) {
			tail_ = (tail_ + 1) % size_;

		}
		else {
			firstWrite_ = false;
		}
	}

	inline T GetOldest()
	{
		return buffer_[tail_];
	}

	uint64_t GetOldestIndex()
	{
		return tail_;
	}

	inline T GetNewest()
	{
		return buffer_[head_];
	}

	inline bool PopFirstOlderThan(uint64_t timestamp, T *sample)
	{
		// start looking from newest observation data
		for (uint64_t i = 0; i < size_; i++) {
			uint64_t index = (head_ - i);
			index = index < 0 ? size_ + index : index;

			if (timestamp >= buffer_[index].time_us && timestamp - buffer_[index].time_us < 100000) {

				// TODO Re-evaluate the static cast and usage patterns
				memcpy(static_cast<void *>(sample), static_cast<void *>(&buffer_[index]), sizeof(*sample));

				// Now we can set the tail to the item which comes after the one we removed
				// since we don't want to have any older data in the buffer
				if (index == static_cast<uint64_t>(head_)) {
					tail_ = head_;
					firstWrite_ = true;

				}
				else {
					tail_ = (index + 1) % size_;
				}

				buffer_[index].time_us = 0;

				return true;
			}

			if (index == static_cast<uint64_t>(tail_)) {
				// we have reached the tail and haven't got a match
				return false;
			}
		}

		return false;
	}

	T &operator[](uint64_t index)
	{
		return buffer_[index];
	}

	// return data at the specified index
	inline T GetFromIndex(uint64_t index)
	{
		if (index >= size_) {
			index = size_ - 1;
		}
		return buffer_[index];
	}

	// push data to the specified index
	inline void PushToIndex(uint64_t index, T sample)
	{
		if (index >= size_) {
			index = size_ - 1;
		}
		buffer_[index] = sample;
	}

	// return the length of the buffer
	uint64_t GetLength()
	{
		return size_;
	}

private:
	T * buffer_;
	uint64_t head_, tail_, size_;
	bool firstWrite_;
};
