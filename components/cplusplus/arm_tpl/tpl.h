#pragma once
#ifndef __cplusplus
void ARMTPLInit();
#else
#include "rtthread.h"
//#include "task.h"
//#include "semphr.h"

#define TickType_t rt_tick_t
#define portMAX_DELAY 1000
#define pdTRUE 0
#define SemaphoreHandle_t rt_sem_t
enum MutexType
{
  NORMAL,
  RECURSIVE,
};

struct MutexStruct
{
  rt_mutex_t mutex;
  MutexType type;
};

struct ThreadStruct
{
  rt_thread_t task;
  void* (*func)(void*);
  void* arg;
  rt_sem_t joinSemaphore;
  rt_sem_t detachSemaphore;
};

class ConditionVariable
{
public:
  ConditionVariable();
  ~ConditionVariable();
  void wait(rt_mutex_t lock, bool recursive);
  int timedWait(rt_mutex_t lock, bool recursive, unsigned int timeoutMS);
  void signal();
  void broadcast();
private:
  SemaphoreHandle_t s;
  SemaphoreHandle_t h;
  rt_mutex_t x;
};

#endif
