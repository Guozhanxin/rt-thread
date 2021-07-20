#include <arm-tpl.h>
#include "tpl.h"
#include <cstdio>

static unsigned int lts_cnt = 0;

static void cpp_thread_entry(void *arg)
{
    for (unsigned int i = 0; i < RT_THREAD_TLS_MAX; i++)
    {
        rt_thread_tls_put(nullptr, i, nullptr);
    }
    arm_tpl_thread_struct *arm_tpl_tid = (arm_tpl_thread_struct *)arg;
    arm_tpl_tid->func(arm_tpl_tid->arg);
    rt_sem_release(arm_tpl_tid->join_sem);
    while (rt_sem_take(arm_tpl_tid->detach_sem, ARM_TPL_MAX_DELAY) != 0);

    for (volatile unsigned int i = 0; i < lts_cnt; i++)
    {
        unsigned int k = 2 * i;
        void *val = rt_thread_tls_get(nullptr, k);
        if (val != nullptr)
        {
            void (*__at_exit)(void *) = (void (*)(void *))(rt_thread_tls_get(nullptr, k + 1));
            if (__at_exit != nullptr)
                __at_exit(val);
        }
    }

    rt_sem_delete(arm_tpl_tid->detach_sem);
    rt_sem_delete(arm_tpl_tid->join_sem);
    rt_free((void *)arm_tpl_tid);
}

extern "C" int __ARM_TPL_thread_create(__ARM_TPL_thread_t *__t,
                                       void *(*__func)(void *),
                                       void *__arg)
{
    char name[8] = "thxx";
    static int index = 0;
    sprintf(name, "%s%d", "thxx", index++);
    arm_tpl_thread_struct *arm_tpl_tid = (arm_tpl_thread_struct *)rt_malloc(sizeof(arm_tpl_thread_struct));
    if (arm_tpl_tid == nullptr)
        goto exit1;
    arm_tpl_tid->arg = __arg;
    arm_tpl_tid->func = __func;
    arm_tpl_tid->join_sem = rt_sem_create("semx", 0, RT_IPC_FLAG_PRIO);
    if (arm_tpl_tid->join_sem == nullptr)
        goto exit2;
    arm_tpl_tid->detach_sem = rt_sem_create("semx", 0, RT_IPC_FLAG_PRIO);
    if (arm_tpl_tid->detach_sem == nullptr)
        goto exit3;
    if ((arm_tpl_tid->task = rt_thread_create(name, cpp_thread_entry, (void *)arm_tpl_tid, ARM_TPL_THREAD_STACK_SIZE, FINSH_THREAD_PRIORITY, 100)) != 0)
    {
        rt_thread_startup(arm_tpl_tid->task);
        __t->data = (std::uintptr_t)arm_tpl_tid;
        return 0;
    }
exit:
    rt_sem_delete(arm_tpl_tid->detach_sem);
exit3:
    rt_sem_delete(arm_tpl_tid->join_sem);
exit2:
    rt_free(arm_tpl_tid);
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
    return (__ARM_TPL_thread_id)rt_thread_self();
}

extern "C" __ARM_TPL_thread_id __ARM_TPL_thread_get_id(
    const __ARM_TPL_thread_t *__t)
{
    return (__ARM_TPL_thread_id)(((arm_tpl_thread_struct *)(__t->data))->task);
}

extern "C" int __ARM_TPL_thread_join(__ARM_TPL_thread_t *__t)
{
    arm_tpl_thread_struct *arm_tpl_tid = (arm_tpl_thread_struct *)(__t->data);
    rt_sem_take(arm_tpl_tid->join_sem, RT_WAITING_FOREVER);
    rt_sem_release(arm_tpl_tid->detach_sem);
    return 0;
}

extern "C" int __ARM_TPL_thread_detach(__ARM_TPL_thread_t *__t)
{
    arm_tpl_thread_struct *arm_tpl_tid = (arm_tpl_thread_struct *)(__t->data);
    rt_sem_release(arm_tpl_tid->detach_sem);
    return 0;
}

extern "C" void __ARM_TPL_thread_yield()
{
    rt_thread_yield();
}

extern "C" int __ARM_TPL_thread_nanosleep(const __ARM_TPL_timespec_t *__req,
        __ARM_TPL_timespec_t *__rem)
{
    rt_tick_t tick = __req->tv_sec * RT_TICK_PER_SECOND + (__req->tv_nsec * RT_TICK_PER_SECOND) / 1000000000;
    rt_thread_delay(tick);
    // FIXME
    if (__rem != nullptr)
    {
        __rem->tv_sec = 0;
        __rem->tv_nsec = 0;
    }
    return 0;
}

extern "C" unsigned __ARM_TPL_thread_hw_concurrency()
{
    return 1;
}

extern "C" int __ARM_TPL_tls_create(__ARM_TPL_tls_key *__key,
                                    void (*__at_exit)(void *))
{
    if (lts_cnt > RT_THREAD_TLS_MAX / 2) return -1;
    *__key = lts_cnt;
    unsigned int k = 2 * *__key ;
    rt_thread_tls_put(NULL, k, nullptr);
    rt_thread_tls_put(NULL, k + 1, (void *) __at_exit);
    lts_cnt++;
    return 0;
}

extern "C" void *__ARM_TPL_tls_get(__ARM_TPL_tls_key __key)
{
    if (__key >= lts_cnt) return nullptr;
    unsigned int k = 2 * __key;
    return rt_thread_tls_get(nullptr, k);
}

extern "C" int __ARM_TPL_tls_set(__ARM_TPL_tls_key __key, void *__p)
{
    if (__key >= lts_cnt) return -1;
    unsigned int k = 2 * __key;
    rt_thread_tls_put(nullptr, k, __p);
    return 0;
}
