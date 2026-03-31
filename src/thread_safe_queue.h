#ifndef FASTCAT_THREAD_SAFE_QUEUE_H_
#define FASTCAT_THREAD_SAFE_QUEUE_H_

#include <queue>

#include "fastcat/mutex.h"

namespace fastcat {
template <class T>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue(void) {}

  ~ThreadSafeQueue(void) {}

  void push(T t)
  {
    LockGuard<Mutex> lock(m);
    q.push(t);
  }

  T front(void)
  {
    LockGuard<Mutex> lock(m);
    return q.front();
  }

  void pop(void)
  {
    LockGuard<Mutex> lock(m);
    q.pop();
  }

  bool empty(void)
  {
    LockGuard<Mutex> lock(m);
    return q.empty();
  }

  bool try_pop(T& item)
  {
    LockGuard<Mutex> lock(m);
    if (q.empty()) {
      return false;
    }
    item = q.front();
    q.pop();
    return true;
  }
  

private:
  std::queue<T> q;
  mutable Mutex  m;

};

}
#endif
