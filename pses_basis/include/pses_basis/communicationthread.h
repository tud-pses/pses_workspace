#ifndef COMMUNICATIONTHREAD_H
#define COMMUNICATIONTHREAD_H

#include <condition_variable>
#include <mutex>
#include <thread>

class CommunicationThread
{
public:
  bool active;
  std::thread worker;

  inline CommunicationThread()
  {
    lock = std::unique_lock<std::mutex>(m);
    notified = false;
    active = false;
  }

  // Move initialization
  CommunicationThread(CommunicationThread&& other) = delete;

  // Copy initialization
  CommunicationThread(const CommunicationThread& other) = delete;

  // Move assignment
  CommunicationThread& operator=(CommunicationThread&& other) = delete;

  // Copy assignment
  CommunicationThread& operator=(const CommunicationThread& other)
  {
    std::lock(m, other.m);
    std::lock_guard<std::mutex> self_lock(m, std::adopt_lock);
    std::lock_guard<std::mutex> other_lock(other.m, std::adopt_lock);
    active = other.active;
    notified = other.notified;
    lock = std::unique_lock<std::mutex>(m);
    return *this;
  }

  virtual void startThread() = 0;

  virtual void stopThread() = 0;

  virtual void workerFunction() = 0;

  inline void sleep()
  {
    notified = false;
    while (!notified) // loop to avoid spurious wakeups
    {
      cond_var.wait(lock);
    }
  }

  inline void wakeUp()
  {
    notified = true;
    cond_var.notify_one();
  }

private:
  mutable std::mutex m;
  std::condition_variable cond_var;
  std::unique_lock<std::mutex> lock;
  bool notified;
};

#endif // COMMUNICATIONTHREAD_H
