#pragma once

#include <thread>
#include <memory>
#include <iostream>


class WorkerThread
{

public:

	WorkerThread(int s_x, int s_y, int e_x, int e_y, int id);
	~WorkerThread();

	/*
	A thread can not be copied.
	*/
	WorkerThread(WorkerThread const&) = delete;
	WorkerThread& operator=(WorkerThread const&) = delete;

	WorkerThread(WorkerThread && other);
	WorkerThread& operator = (WorkerThread&& other);

	void assign_work(std::thread & t);
	void join();

	void set_indicies(int new_start_x, int new_start_y, int new_end_x, int new_end_y);
	
	
	inline int get_start_x() { return _start_x; }
	inline int get_start_y() { return _start_y; }

	inline int get_end_x() { return _end_x; }
	inline int get_end_y() { return _end_y; }

	inline int get_worker_id() { return _worker_id; }

	static unsigned int max_threads();

private:

	std::thread _t;

	int _start_x;
	int _start_y;
	
	int _end_x;
	int _end_y;

	int _worker_id;
};