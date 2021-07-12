#include <arm-tpl.h>
#include <cstdint>
#include <stdatomic.h>
#include "tpl.h"

static int checkCreate(volatile __ARM_TPL_mutex_t* __vm, bool recursive = false)
{
  if (__vm->data == 0)
  {
    uintptr_t mut_null = 0;
    MutexStruct* mutexStructPtr = (MutexStruct*)rt_malloc(sizeof(MutexStruct));
    if (mutexStructPtr == nullptr)
      return -1;
    if (recursive)
      mutexStructPtr->mutex = rt_mutex_create("mutexx", RT_IPC_FLAG_PRIO);
    else
      mutexStructPtr->mutex = rt_mutex_create("mutexx", RT_IPC_FLAG_PRIO);
    if (mutexStructPtr->mutex == nullptr)
    {
      rt_free(mutexStructPtr);
      return -1;
    }
    mutexStructPtr->type = recursive? RECURSIVE: NORMAL;
    uintptr_t mut_new = reinterpret_cast<uintptr_t>(mutexStructPtr);
    if (!atomic_compare_exchange_strong(&__vm->data, &mut_null, mut_new))
    {
      rt_mutex_delete(mutexStructPtr->mutex);
      rt_free(mutexStructPtr);
    }
	}
  return 0;
}

static int mutexLock(MutexStruct* mutexStructPtr, TickType_t timeOut)
{
  if (mutexStructPtr->type == RECURSIVE)
  {
    if (rt_mutex_take(mutexStructPtr->mutex, timeOut) == pdTRUE)
      return 0;
  }
  else
  {
    if (rt_mutex_take(mutexStructPtr->mutex, timeOut) == pdTRUE)
      return 0;
  }
  return -1;
}

static int mutexUnlock(MutexStruct* mutexStructPtr)
{
  if (mutexStructPtr->type == RECURSIVE)
    rt_mutex_release(mutexStructPtr->mutex);
  else
    rt_mutex_release(mutexStructPtr->mutex);
  return 0;
}

extern "C" int __ARM_TPL_recursive_mutex_init(__ARM_TPL_mutex_t* __m)
{
  volatile __ARM_TPL_mutex_t *__vm = __m;
  return checkCreate(__vm, true);
}

extern "C" int __ARM_TPL_mutex_lock(__ARM_TPL_mutex_t* __m) {
	volatile __ARM_TPL_mutex_t *__vm = __m;
	if (checkCreate(__vm))
    return -1;
  while (mutexLock((MutexStruct*)(__vm->data), portMAX_DELAY) != 0);
  return 0;
}

extern "C" int __ARM_TPL_mutex_trylock(__ARM_TPL_mutex_t* __m)
{
  volatile __ARM_TPL_mutex_t *__vm = __m;
	if (checkCreate(__vm))
    return -1;
  return mutexLock((MutexStruct*)(__vm->data), 0);
}

extern "C" int __ARM_TPL_mutex_unlock(__ARM_TPL_mutex_t* __m)
{
  volatile __ARM_TPL_mutex_t* __vm = __m;
  return mutexUnlock((MutexStruct*)(__vm->data));
}

extern "C" int __ARM_TPL_mutex_destroy(__ARM_TPL_mutex_t* __m)
{
  volatile __ARM_TPL_mutex_t *__vm = __m;
  if (__vm->data != 0)
  {
    rt_mutex_delete(((MutexStruct*)(__vm->data))->mutex);
    rt_free((void*)(__vm->data));
    __vm->data = 0;
  }
  return 0;
}
