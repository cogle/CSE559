#pragma once

#include <atomic>

class SpinlockMutex
{
public:
	
	SpinlockMutex()
	{
		_flag.clear();
	}
	
	~SpinlockMutex()
	{
	
	}

	SpinlockMutex(const SpinlockMutex &) = delete;
	SpinlockMutex(const SpinlockMutex &&) = delete;
	SpinlockMutex & operator=(const SpinlockMutex&) = delete;

	void lock()
	{
		while (_flag.test_and_set(std::memory_order_acquire));
	}

	void unlock()
	{
		_flag.clear(std::memory_order_release);
	}

private: 
	std::atomic_flag _flag;
};