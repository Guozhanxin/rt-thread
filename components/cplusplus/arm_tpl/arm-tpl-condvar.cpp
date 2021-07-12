#include <arm-tpl.h>
#include "tpl.h"
#include <new>
#include <cstdint>
#include <stdatomic.h>

ConditionVariable::ConditionVariable()
{
  s = rt_sem_create("semxs", 0, RT_IPC_FLAG_PRIO);
  if (s == nullptr)
    RT_ASSERT(0);
  h = rt_sem_create("semxh", 0, RT_IPC_FLAG_PRIO);
  if (h == nullptr)
  {
    rt_sem_delete(s);
    RT_ASSERT(0);
  }
  x = rt_mutex_create("mutx", RT_IPC_FLAG_PRIO);// xSemaphoreCreateMutex();
  if (x == nullptr)
  {
    rt_sem_delete(s);
    rt_sem_delete(h);
    RT_ASSERT(0);
  }
}

ConditionVariable::~ConditionVariable()
{
  rt_mutex_delete(x);
  rt_sem_delete(h);
  rt_sem_delete(s);
}

void ConditionVariable::wait(rt_mutex_t lock, bool recursive)
{
  while (rt_mutex_take(x, portMAX_DELAY) != pdTRUE);
  rt_sem_release(s);
  rt_mutex_release(x);
  if (recursive)
    rt_mutex_release(lock);
  else
    rt_mutex_release(lock);
  while (rt_sem_take(h, portMAX_DELAY) != pdTRUE);
  if (recursive)
    while (rt_mutex_take(lock, portMAX_DELAY) != pdTRUE);
  else
    while (rt_mutex_take(lock, portMAX_DELAY) != pdTRUE);
}

int ConditionVariable::timedWait(rt_mutex_t lock, bool recursive, unsigned int timeoutMS)
{
  int toBeReturned = 0;
  while (rt_mutex_take(x, portMAX_DELAY) != pdTRUE);
  rt_sem_release(s);
  rt_mutex_release(x);
  if (recursive)
    rt_mutex_release(lock);
  else
    rt_mutex_release(lock);
  if (rt_sem_take(h, rt_tick_from_millisecond( timeoutMS)) != pdTRUE)
  {
    while (rt_mutex_take(x, portMAX_DELAY) != pdTRUE);
    if (rt_sem_take(h, 0) != pdTRUE)
    {
      if (rt_sem_take(s, 0) != pdTRUE)
        toBeReturned = -1;
      else
        toBeReturned = 1;
    }
    rt_mutex_release(x);
  }
  if (recursive)
    while (rt_mutex_take(lock, portMAX_DELAY) != pdTRUE);
  else
    while (rt_mutex_take(lock, portMAX_DELAY) != pdTRUE);
  return toBeReturned;
}

void ConditionVariable::signal()
{
  while (rt_mutex_take(x, portMAX_DELAY) != pdTRUE);
  if (rt_sem_take(s, 0) == pdTRUE)
    rt_sem_release(h);
  rt_mutex_release(x);
}

void ConditionVariable::broadcast()
{
  while (rt_mutex_take(x, portMAX_DELAY) != pdTRUE);
  auto count = s->value;// uxSemaphoreGetCount(s);
  for (auto i = 0; i < count; i++)
  {
    while (rt_sem_take(s, portMAX_DELAY) != pdTRUE);
    rt_sem_release(h);
  }
  rt_mutex_release(x);
}

static int checkCreate(volatile __ARM_TPL_condvar_t* __vcv)
{
  if (__vcv->data == 0)
  {
    uintptr_t cv_new;
//    try
    {
      cv_new = reinterpret_cast<uintptr_t>(new ConditionVariable());
    }
    if (cv_new == 0)
    {
      return -1;
    }
    uintptr_t cv_null = 0;
    if (!atomic_compare_exchange_strong(&__vcv->data, &cv_null, cv_new))
      delete reinterpret_cast<ConditionVariable*>(cv_new);
  }
  return 0;
}

extern "C" int __ARM_TPL_condvar_wait(__ARM_TPL_condvar_t* __cv, __ARM_TPL_mutex_t* __m)
{
  volatile __ARM_TPL_condvar_t* __vcv = __cv;
  if (checkCreate(__vcv) != 0)
    return -1;
  struct MutexStruct* mS = (struct MutexStruct*)(__m->data);
  ((ConditionVariable*) __vcv->data)->wait(mS->mutex, mS->type == RECURSIVE);
  return 0;
}

extern "C" 
    int __ARM_TPL_condvar_timedwait(__ARM_TPL_condvar_t* __cv,
                                __ARM_TPL_mutex_t* __m,
                                __ARM_TPL_timespec_t* __ts)
{
  volatile __ARM_TPL_condvar_t* __vcv = __cv;
  if (checkCreate(__vcv) != 0)
    return -1;
  __ARM_TPL_timespec_t now;
  if (__ARM_TPL_clock_realtime(&now) != 0)
    return -1;
  struct MutexStruct* mS = (struct MutexStruct*)(__m->data);
  unsigned int timeoutMS = (__ts->tv_sec - now.tv_sec) * 1000 + (__ts->tv_nsec - now.tv_nsec) / 1000000;
  if (((ConditionVariable*) __vcv->data)->timedWait(mS->mutex, mS->type == RECURSIVE, timeoutMS) < 0)
    return -1;
  return 0;
}

extern "C" int __ARM_TPL_condvar_signal(__ARM_TPL_condvar_t* __cv)
{
  volatile __ARM_TPL_condvar_t *__vcv = __cv;
  if (__vcv->data != 0)
    ((ConditionVariable*) __vcv->data)->signal();
  return 0;
}

extern "C" int __ARM_TPL_condvar_broadcast(__ARM_TPL_condvar_t* __cv)
{
  volatile __ARM_TPL_condvar_t *__vcv = __cv;
  if (__vcv->data != 0)
    ((ConditionVariable*) __vcv->data)->broadcast();
  return 0;
}

extern "C" int __ARM_TPL_condvar_destroy(__ARM_TPL_condvar_t* __cv)
{
  volatile __ARM_TPL_condvar_t *__vcv = __cv;
  if (__vcv->data != 0)
  {
    delete (ConditionVariable*) __vcv->data;
    __vcv->data = 0;
  }
  return 0;
}
