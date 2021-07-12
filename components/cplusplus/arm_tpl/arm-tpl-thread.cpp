#include <arm-tpl.h>
#include "tpl.h"
#include <cstdio>
static unsigned int localStorageKeyCounter = 0;

static void cTask(void* arg)
{
  for (unsigned int i = 0; i < RT_THREAD_TLS_MAX; i++)
    rt_thread_tls_put(nullptr, i, nullptr);
  ThreadStruct* threadStructPtr = (ThreadStruct*)arg;
  threadStructPtr->func(threadStructPtr->arg);
  rt_sem_release(threadStructPtr->joinSemaphore);
  while (rt_sem_take(threadStructPtr->detachSemaphore, portMAX_DELAY) != pdTRUE);
  for (volatile unsigned int i = 0; i < localStorageKeyCounter; i++)
  {
    unsigned int k = 2 * i;
    void* val = rt_thread_tls_get(nullptr, k);
    if (val != nullptr)
    {
      void (*__at_exit)(void*) = (void (*)(void*))(rt_thread_tls_get(nullptr, k + 1));
      if (__at_exit != nullptr)
        __at_exit(val);
    }
  }
  
  rt_sem_delete(threadStructPtr->detachSemaphore);
  rt_sem_delete(threadStructPtr->joinSemaphore);
  rt_free((void*)threadStructPtr);
//  vTaskDelete(nullptr);
//  for(;;);
}

extern "C" int __ARM_TPL_thread_create(__ARM_TPL_thread_t* __t,
                            void* (*__func)(void*),
                            void* __arg)
{
  ThreadStruct* threadStructPtr = (ThreadStruct*)rt_malloc(sizeof(ThreadStruct));
  if (threadStructPtr == nullptr)
    goto exit1;
  threadStructPtr->arg = __arg;
  threadStructPtr->func = __func;
  threadStructPtr->joinSemaphore = rt_sem_create("semx", 0, RT_IPC_FLAG_PRIO);
  if (threadStructPtr->joinSemaphore == nullptr)
    goto exit2;
  threadStructPtr->detachSemaphore = rt_sem_create("semx", 0, RT_IPC_FLAG_PRIO);
  if (threadStructPtr->detachSemaphore == nullptr)
    goto exit3;
  if ((threadStructPtr->task = rt_thread_create("threadx", cTask, (void*)threadStructPtr, 1024, RT_THREAD_PRIORITY_MAX -1, 100)) != 0)
//  if (xTaskCreate((TaskFunction_t)cTask, "C++", configMINIMAL_STACK_SIZE, (void*)threadStructPtr, tskIDLE_PRIORITY, &(threadStructPtr->task)) == pdTRUE)
  {
    rt_thread_startup(threadStructPtr->task);
    __t->data = (std::uintptr_t)threadStructPtr;
    return 0;
  }
  exit:
    rt_sem_delete(threadStructPtr->detachSemaphore);
  exit3:
    rt_sem_delete(threadStructPtr->joinSemaphore);
  exit2:
    rt_free(threadStructPtr);
  exit1:
    return -1;
}

extern "C" int __ARM_TPL_thread_id_compare(__ARM_TPL_thread_id __tid1,
                                 __ARM_TPL_thread_id __tid2)
{
  if (__tid1 > __tid2)
    return 1;
  else if (__tid1 < __tid2)
    return -1;
  else
    return 0;
}

extern "C" __ARM_TPL_thread_id __ARM_TPL_thread_get_current_id()
{
//  return (__ARM_TPL_thread_id)xTaskGetCurrentTaskHandle();
    return (__ARM_TPL_thread_id)rt_thread_self();
}

extern "C" __ARM_TPL_thread_id __ARM_TPL_thread_get_id(
                    const __ARM_TPL_thread_t* __t)
{
    return (__ARM_TPL_thread_id)(((ThreadStruct*)(__t->data))->task);
}

extern "C" int __ARM_TPL_thread_join(__ARM_TPL_thread_t* __t)
{
  ThreadStruct* threadStructPtr = (ThreadStruct*)(__t->data);
    rt_sem_take(threadStructPtr->joinSemaphore, RT_WAITING_FOREVER);
    rt_sem_release(threadStructPtr->detachSemaphore);
//  while (xSemaphoreTake(threadStructPtr->joinSemaphore, portMAX_DELAY) != pdTRUE);
//  xSemaphoreGive(threadStructPtr->detachSemaphore);
  return 0;
}

extern "C" int __ARM_TPL_thread_detach(__ARM_TPL_thread_t* __t)
{
  ThreadStruct* threadStructPtr = (ThreadStruct*)(__t->data);
    rt_sem_release(threadStructPtr->detachSemaphore);
//  xSemaphoreGive(threadStructPtr->detachSemaphore);
  return 0;
}

extern "C" void __ARM_TPL_thread_yield()
{
  rt_thread_yield();
}

extern "C" int __ARM_TPL_thread_nanosleep(const __ARM_TPL_timespec_t *__req,
                               __ARM_TPL_timespec_t *__rem)
{
//  vTaskDelay(__req->tv_sec * configTICK_RATE_HZ +
//             __req->tv_nsec /1e6 * portTICK_RATE_MS);
    rt_tick_t tick = __req->tv_sec * RT_TICK_PER_SECOND + (__req->tv_nsec * RT_TICK_PER_SECOND)/ 1000000000;
    rt_thread_delay(tick);
  // FIXME
  if (__rem != nullptr)
  {
    __rem->tv_sec = 0;
    __rem->tv_nsec = 0;
  }
  return 0;
}

extern "C" unsigned __ARM_TPL_thread_hw_concurrency() {
  return 1;
}

extern "C" int __ARM_TPL_tls_create(__ARM_TPL_tls_key* __key,
                         void (*__at_exit)(void*))
{
  if (localStorageKeyCounter > RT_THREAD_TLS_MAX / 2)
    return -1;
  *__key = localStorageKeyCounter;
  unsigned int k = 2 * *__key ;
  rt_thread_tls_put(NULL, k, nullptr);
  rt_thread_tls_put(NULL, k + 1, (void*) __at_exit);
  localStorageKeyCounter++;
  return 0;
}

extern "C" void* __ARM_TPL_tls_get(__ARM_TPL_tls_key __key)
{
  if (__key >= localStorageKeyCounter)
    return nullptr;
  unsigned int k = 2 * __key;
  return rt_thread_tls_get(nullptr, k);
//  return 0;
}

extern "C" int __ARM_TPL_tls_set(__ARM_TPL_tls_key __key, void* __p)
{
  if (__key >= localStorageKeyCounter)
    return -1;
  unsigned int k = 2 * __key;
  rt_thread_tls_put(nullptr, k, __p);
  return 0;
}
