#ifndef  MUTEX__H
#define  MUTEX__H

#include <boost/thread/thread.hpp>

/** pthread_mutex_t cpp wrapper */
class Mutex
{
public:
    Mutex() { pthread_mutex_init(&m_mutex, NULL); }
    void lock() { pthread_mutex_lock(&m_mutex); }
    void unlock() { pthread_mutex_unlock(&m_mutex); }
 
    // Create and destroy a mutex in a scope using a constructor/destructor.
    class ScopedLock
    {
    public:
        ScopedLock(Mutex &mutex) : _mutex(mutex) { _mutex.lock(); }
        ~ScopedLock() { _mutex.unlock(); }
    private:
        Mutex &_mutex;
    };

private:
    pthread_mutex_t m_mutex;
};

#endif // MUTEX__H
