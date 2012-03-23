#ifndef LOCKED_QUEUE_H_
#define LOCKED_QUEUE_H_

#include <ros/ros.h>
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

  ~lockedQueue()
  {
    ros::Rate naptime(10);
    while(pthread_mutex_destroy(&the_mutex) == EBUSY)
    {
      naptime.sleep();
    }

  }

  void push(const Data& data)
  {
    pthread_mutex_lock(&the_mutex);
    the_queue.push(data); // push the data
    pthread_mutex_unlock(&the_mutex);
  }

  bool empty()
  {
    pthread_mutex_lock(&the_mutex);
    bool result = the_queue.empty(); // push the data
    pthread_mutex_unlock(&the_mutex);
    return result;
  }

  Data& front()
  {
    pthread_mutex_lock(&the_mutex);
    Data& data = the_queue.front(); // push the data
    pthread_mutex_unlock(&the_mutex);
    return data;
  }
    
  Data const& front() const
  {
    pthread_mutex_lock(&the_mutex);
    Data const& data = the_queue.front(); // push the data
    pthread_mutex_unlock(&the_mutex);
    return data;
  }

  void pop()
  {
    pthread_mutex_lock(&the_mutex);
    the_queue.pop(); // push the data
    pthread_mutex_unlock(&the_mutex);
  }

  unsigned int size()
  {
    pthread_mutex_lock(&the_mutex);
    unsigned int result = the_queue.size();
    pthread_mutex_lock(&the_mutex);
    return result;
  }
};

#endif // LOCKED_QUEUE_H_ 
