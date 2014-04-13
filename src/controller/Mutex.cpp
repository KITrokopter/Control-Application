#include "Mutex.hpp"

/*
 * Constructor
 */
Mutex::Mutex()
{
	pthread_mutex_init(&m_mutex, NULL);
}

/*
 * Locking mutex
 */
void Mutex::lock()
{
	pthread_mutex_lock(&m_mutex);
}

/*
 * Unlocking mutex
 */
void Mutex::unlock()
{
	pthread_mutex_unlock(&m_mutex);
}
