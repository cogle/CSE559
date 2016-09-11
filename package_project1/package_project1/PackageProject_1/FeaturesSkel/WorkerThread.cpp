#include "WorkerThread.h"

WorkerThread::WorkerThread(int s_x, int s_y, int e_x, int e_y, int id) : _start_x(s_x), _start_y(s_y), _end_x(e_x), _end_y(e_y), _worker_id(id)
{

}

WorkerThread::~WorkerThread()
{

}

WorkerThread::WorkerThread(WorkerThread && other)
{
	_start_x = other._start_x;
	_start_y = other._start_y;
	
	_end_x = other._end_x;
	_end_y = other._end_y;
	
	_worker_id = other._worker_id;

	_t = std::move(other._t);



}

WorkerThread& WorkerThread::operator = (WorkerThread&& other)
{
	
	if (this != &other)
	{
		_start_x = other._start_x;
		_start_y = other._start_y;

		_end_x = other._end_x;
		_end_y = other._end_y;

		_worker_id = other._worker_id;

		_t = std::move(other._t);
	}

	return *this;
}

void WorkerThread::assign_work(std::thread & t)
{
	_t = std::move(t);
}

void WorkerThread::join()
{
	_t.join();
}

unsigned int WorkerThread::max_threads()
{
	return std::thread::hardware_concurrency();
}

void WorkerThread::set_indicies(int new_start_x, int new_start_y, int new_end_x, int new_end_y)
{
	_start_x = new_start_x;
	_start_y = new_start_y;

	_end_x = new_end_x;
	_end_y = new_end_y;
}
