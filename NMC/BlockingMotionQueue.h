#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>


template <class T>
class MotionQueue
{
private:
	std::queue<T> m_queue;						//queue
	std::mutex m_mutex;							//mutex
	std::condition_variable_any m_notEmpty;		//condition_variable notempty
	std::condition_variable_any m_notFull;		//condition_variable notfull
	int m_maxSize;								//blockingqueue max size

private:	
	bool IsFull()				//the queue is or not full
	{
		return m_queue.size() == m_maxSize;
	}

	bool IsEmpty()				//the queue is or not empty
	{
		return m_queue.empty();
	}

public:
	bool Put(T &x)				//Put x into the BlockingMotionQueue
	{
		std::lock_guard<std::mutex> locker(m_mutex);
		while (IsFull())
		{
			//std::cout << "the blocking queue is full,waiting..." << std::endl;
			m_notFull.wait(m_mutex);			//the Queue is blocking when the queue is full
		}
		m_queue.push(x);
		m_notEmpty.notify_one();
		return true;
	}

	bool Take(T &x)				//Take out x from the BlockingMotionQueue
	{
		std::lock_guard<std::mutex> locker(m_mutex);

		while (IsEmpty())
		{
			//std::cout << "the blocking queue is empty,wating..." << std::endl;
			m_notEmpty.wait(m_mutex);			//the Queue is blocking when the queue is empty
		}

		x = m_queue.front();
		m_queue.pop();
		m_notFull.notify_one();
		return true;
	}

	bool Empty()
	{
		while (m_queue.empty() == false)
		{
			m_queue.pop();
		}

		return true;
	}

	bool isEmpty()
	{
		return m_queue.empty();
	}

public:


	MotionQueue()
	{
		this->m_maxSize = 8096;

	}
	~MotionQueue() {};

};

class Motion
{
private:
	int m_axis;
	double m_range;
	double m_speed;

public:
	Motion(int axis, double range, double speed)
	{
		m_axis = axis;
		m_range = range;
		m_speed = speed;
	}
	~Motion() {}

	int getAxis() { return m_axis; }
	double getRange() { return m_range; }
	double getSpeed() { return m_speed; }
};

