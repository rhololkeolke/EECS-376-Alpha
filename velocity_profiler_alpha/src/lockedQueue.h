#ifndef LOCKED_QUEUE_H_
#define LOCKED_QUEUE_H_

#include <queue>
#include <pthread.h>

template<typename Data>
class lockedQueue
{
 private:
  std::queue<Data> the_queue;
  pthread_mutex_t the_mutex;
 public:
  lockedQueue()
  {
    pthread_mutex_init(&the_mutex, NULL); // initialize the mutex with default attributes
  }

  void push(const Data& data)
  {
    int err;
    if((err=pthread_mutex_lock(&the_mutex))!=0) // lock the queue, or block until lockable
    {
      throw err;
    }
    the_queue.push(data); // push the data
    if((err=pthread_mutex_unlock(&the_mutex))!=0) // unlock the queue for other threads to use
    {
      throw err;
    }
  }

  bool empty()
  {
    int err;
    if((err=pthread_mutex_lock(&the_mutex))!=0) // lock the queue, or block until lockable
    {
      throw err; // something went wrong throw the error
    }
    bool result = the_queue.empty(); // push the data
    if((err=pthread_mutex_unlock(&the_mutex))!=0) // unlock the queue for other threads to use
    {
      throw err; // throw the error
    }
    return result;
  }

  Data& front()
  {
    int err;
    if((err=pthread_mutex_lock(&the_mutex))!=0) // lock the queue, or block until lockable
    {
      throw err;
    }
    Data& data = the_queue.front(); // push the data
    if((err=pthread_mutex_unlock(&the_mutex))!=0) // unlock the queue for other threads to use
    {
      throw err;
    }
    return data;
  }
    
  Data const& front() const
  {
    int err;
    if((err=pthread_mutex_lock(&the_mutex))!=0) // lock the queue, or block until lockable
    {
      throw err;
    }
    Data const& data = the_queue.front(); // push the data
    if((err=pthread_mutex_unlock(&the_mutex))!=0) // unlock the queue for other threads to use
    {
      throw err;
    }
    return data;
  }

  void pop()
  {
    int err;
    if((err=pthread_mutex_lock(&the_mutex))!=0) // lock the queue, or block until lockable
    {
      throw err;
    }
    the_queue.pop(); // push the data
    if((err=pthread_mutex_unlock(&the_mutex))!=0) // unlock the queue for other threads to use
    {
      throw err;
    }
  }

  unsigned int size()
  {
    int err;
    if((err=pthread_mutex_lock(&the_mutex))!=0) // lock the queue, or block until lockable
    {
      throw err;
    }
    unsigned int result = the_queue.size();
    if((err=pthread_mutex_unlock(&the_mutex))!=0) // unlock the queue for other threads to use
    {
      throw err;
    }
    return result;
  }
};

#endif // LOCKED_QUEUE_H_ 
