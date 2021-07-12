
#include <iostream>
#include <mutex>
#include <thread>

#include <condition_variable>
//#include <pthread.h>
#include <finsh.h>

using namespace std;

std::mutex mtx;           // mutex for critical section
std::recursive_mutex rmtx;           // mutex for critical section


void fun(int param, int param1)
{
    mtx.lock();
    {
        // rt_thread_mdelay(100);
        rt_kprintf("fun param: %d, param1:  %d\n", param, param1);
    }
    mtx.unlock();
}

void fun1(int param, int param1)
{
    mtx.lock();
    {
        // rt_thread_mdelay(100);
        rt_kprintf("fun1 param: %d, param1: %d\n", param, param1);
    }
    mtx.unlock();
}


void test_thread_detach(void)
{
    thread t(fun, 50, 100);
    thread t1(fun1, 51, 101);

    t.detach();
    t1.detach();

    cout << "Main thread  !" << endl;
}
MSH_CMD_EXPORT(test_thread_detach, cpp thread detach test);

void test_thread_join(void)
{
    thread t(fun, 50, 100);
    thread t1(fun1, 51, 101);

    t.join();
    t1.join();

    cout << "Main thread  !" << endl;
}
MSH_CMD_EXPORT(test_thread_join, cpp thread join test);


/* mutex lock/unlock/trylock samples */
volatile int counter = 0; // non-atomic counter

void attempt_5k_increases() {
    for (int i=0; i<10000; ++i) {
        ++counter;
    }
}

void attempt_5k_increases_lock() {
    for (int i=0; i<10000; ++i) {
        mtx.lock();
        ++counter;
        mtx.unlock();
    }
}

void attempt_5k_increases_trylock() {
    for (int i=0; i<10000; ++i) {
        if (mtx.try_lock()) {   // only increase if currently not locked:
            ++counter;
            mtx.unlock();
        }
    }
}

int test_mutex_nolock (int argc, const char* argv[]) {
    std::thread *threads[5];
    for (int i=0; i<5; ++i)
        threads[i] = new std::thread(attempt_5k_increases);

    for (int j=0; j<5; ++j) 
        threads[j]->join();

    std::cout << counter << " successful increases of the counter.\n";
    counter = 0;
    return 0;
}
MSH_CMD_EXPORT(test_mutex_nolock, test_mutex_nolock);

int test_mutex_lock (int argc, const char* argv[]) {
    std::thread *threads[5];
    for (int i=0; i<5; ++i)
        threads[i] = new std::thread(attempt_5k_increases_lock);

    for (int j=0; j < 5; ++j) 
        threads[j]->join();

    std::cout << counter << " successful increases of the counter.\n";
    counter = 0;
    return 0;
}

MSH_CMD_EXPORT(test_mutex_lock, test_mutex_lock);

int test_mutex_trylock (int argc, const char* argv[]) {
    std::thread *threads[5];

    for (int i=0; i<5; ++i)
        threads[i] = new std::thread(attempt_5k_increases_trylock);

    for (int j=0; j<5; ++j) 
        threads[j]->join();

    std::cout << counter << " successful increases of the counter.\n";
    counter = 0;
    return 0;
}

MSH_CMD_EXPORT(test_mutex_trylock, test_mutex_trylock);


/* mutex timed_mutex  try_lock_for sample */
std::timed_mutex tmtx;

static void print_look_for()
{
    // waiting to get a lock: each thread prints "-" every 200ms:
    while (!tmtx.try_lock_for(std::chrono::milliseconds(200)))
    {
        std::cout << "-";
    }
    // got a lock! - wait for 1s, then this thread prints "*"
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    std::cout << "*\n";
    tmtx.unlock();
}

int test_mutex_time_for(void)
{
    std::thread *threads[5];

    for (int i = 0; i < 5; ++i)
        threads[i] = new std::thread(print_look_for);

    for (int j = 0; j < 5; ++j)
        threads[j]->join();

    return 0;
}
MSH_CMD_EXPORT(test_mutex_time_for, test_mutex_time);

/* mutex timed_mutex try_lock_until sample */
#include <ctime>

std::timed_mutex cinderella;

static std::chrono::time_point<std::chrono::system_clock> midnight()
{
    using std::chrono::system_clock;
    std::time_t tt = system_clock::to_time_t(system_clock::now());
    struct std::tm *ptm = std::localtime(&tt);
    ++ptm->tm_mday;
    ptm->tm_hour = 0;
    ptm->tm_min = 0;
    ptm->tm_sec = 0;
    return system_clock::from_time_t(mktime(ptm));
}

static void print_carriage_time()
{
    if (cinderella.try_lock_until(midnight()))
    {
        std::cout << "ride back home on carriage\n";
        cinderella.unlock();
    }
    else
        std::cout << "carriage reverts to pumpkin\n";
}

static void print_ball_time()
{
    cinderella.lock();
    std::cout << "at the ball...\n";
    cinderella.unlock();
}

int test_mutex_time_until(void)
{
    std::thread th1(print_ball_time);
    std::thread th2(print_carriage_time);

    th1.join();
    th2.join();

    return 0;
}
MSH_CMD_EXPORT(test_mutex_time_until, test_mutex_time);

/* mutex lock_guard sample */
void print_even_guard(int x)
{
    if (x % 2 == 0)
        std::cout << x << " is even\n";
}

void print_thread_id_guard(int id)
{
    // using a local lock_guard to lock mtx guarantees unlocking on destruction / exception:
    std::lock_guard<std::mutex> lck(mtx);
    print_even_guard(id);
}

int test_mutex_look_guard(void)
{
    std::thread *threads[6];

    for (int i = 0; i < 6; ++i)
        threads[i] = new std::thread(print_thread_id_guard, i + 1);

    for (int j = 0; j < 6; ++j)
        threads[j]->join();

    return 0;
}
MSH_CMD_EXPORT(test_mutex_look_guard, test_mutex_look_guard);


/* mutex unique_lock sample */
static void print_block_unique(int n, char c)
{
    // critical section (exclusive access to std::cout signaled by lifetime of lck):
    std::unique_lock<std::mutex> lck(mtx);
    for (int i = 0; i < n; ++i)
    {
        std::cout << c;
    }
    std::cout << '\n';
}

int test_mutex_unique_look(void)
{
    std::thread th1(print_block_unique, 50, '*');
    std::thread th2(print_block_unique, 50, '$');

    th1.join();
    th2.join();

    return 0;
}
MSH_CMD_EXPORT(test_mutex_unique_look, test_mutex_unique_look);


/* mutex call_once sample */
static int winner = 0;
static void set_winner (int x) { winner = x; }
static std::once_flag winner_flag;

static void wait_1000ms(int id)
{
    rt_thread_mdelay(1000);
    // claim to be the winner (only the first such call is executed):
    std::call_once(winner_flag, set_winner, id);
}

int test_mutex_call_once(void)
{
    std::thread *threads[6];
    // spawn 6 threads:
    for (int i = 0; i < 6; ++i)
        threads[i] = new std::thread(wait_1000ms, i + 1);

    std::cout << "waiting for the first among 6 threads to count 6000 ms...\n";

    for (int j = 0; j < 6; ++j)
        threads[j]->join();

    std::cout << "winner thread: " << winner << '\n';

    return 0;
}
MSH_CMD_EXPORT(test_mutex_call_once, test_mutex_call_once);



std::condition_variable cv;
rt_bool_t ready = RT_FALSE;

static void print_id_cv(int id)
{
    std::unique_lock<std::mutex> lck(mtx);
    while (!ready)
        cv.wait(lck);
    std::cout << "thread " << id << '\n';
}

static void go_cv()
{
    std::unique_lock<std::mutex> lck(mtx);
    ready = RT_TRUE;
    cv.notify_all();
}

int test_cv_norify_all()
{
    std::thread *threads[10];
    // spawn 10 threads:
    for (int i = 0; i < 10; ++i)
        threads[i] =  new std::thread(print_id_cv, i);

    std::cout << "10 threads ready to race...\n";
    go_cv(); // go!

    for (int j = 0; j < 10; ++j)
        threads[j]->join();

    ready = RT_FALSE;
    return 0;
}
MSH_CMD_EXPORT(test_cv_norify_all, test_cv_norify_all);

std::condition_variable produce, consume;

static int cargo = 0; // shared value by producers and consumers

static void consumer_one()
{
    std::unique_lock<std::mutex> lck(mtx);
    while (cargo == 0)
        consume.wait(lck);
    std::cout << cargo << '\n';
    cargo = 0;
    produce.notify_one();
}

static void producer_one(int id)
{
    std::unique_lock<std::mutex> lck(mtx);
    while (cargo != 0)
        produce.wait(lck);
    cargo = id;
    consume.notify_one();
}

int test_cv_norify_one()
{
    std::thread *consumers[10], *producers[10];
    // spawn 10 consumers and 10 producers:
    for (int i = 0; i < 10; ++i)
    {
        consumers[i] = new std::thread(consumer_one);
        producers[i] = new std::thread(producer_one, i + 1);
    }

    // join them back:
    for (int i = 0; i < 10; ++i)
    {
        producers[i]->join();
        consumers[i]->join();
    }

    return 0;
}
MSH_CMD_EXPORT(test_cv_norify_one, test_cv_norify_one);

static int cargo1 = 0;

bool shipment_available() { return cargo1 != 0; }

void consume_wait(int n)
{
    for (int i = 0; i < n; ++i)
    {
        std::unique_lock<std::mutex> lck(mtx);
        cv.wait(lck, shipment_available);
        std::cout << cargo1 << '\n';
        cargo1 = 0;
    }
}

int test_cv_norify_wait(void)
{
    std::thread consumer_thread(consume_wait, 10);

    // produce 10 items when needed:
    for (int i = 0; i <10; ++i)
    {
        while (shipment_available())
            std::this_thread::yield();
        std::unique_lock<std::mutex> lck(mtx);
        cargo1 = i + 1;
        cv.notify_one();
    }

    consumer_thread.join();

    return 0;
}
MSH_CMD_EXPORT(test_cv_norify_wait, test_cv_norify_wait);

rt_bool_t ready_exit = RT_FALSE;

static void print_id_exit(int id)
{
    std::unique_lock<std::mutex> lck(mtx);
    while (!ready)
        cv.wait(lck);
    std::cout << "thread " << id << '\n';
}

static void go_exit(void)
{
    std::unique_lock<std::mutex> lck(mtx);
    std::notify_all_at_thread_exit(cv, std::move(lck));
    ready = RT_TRUE;
}

int test_cv_norify_exit(void)
{
    std::thread *threads[10];
    // spawn 10 threads:
    for (int i = 0; i < 10; ++i)
        threads[i] = new std::thread(print_id_exit, i);
    std::cout << "10 threads ready to race...\n";

    std::thread(go_exit).detach(); // go!

    for (int j = 0; j < 10; ++j)
        threads[j]->join();

    return 0;
}
MSH_CMD_EXPORT(test_cv_norify_exit, test_cv_norify_exit);

#include <future>

static rt_bool_t is_prime(int x)
{
    for (int i = 2; i < x; ++i)
        if (x % i == 0)
            return RT_FALSE;
    return RT_TRUE;
}

int test_future_get(void)
{
    // call function asynchronously:
    std::future<rt_bool_t> fut = std::async(is_prime, 194232491);

    std::cout << "checking...\n";
    fut.wait();

    std::cout << "\n194232491 ";
    if (fut.get()) // guaranteed to be ready (and not block) after wait returns
        std::cout << "is prime.\n";
    else
        std::cout << "is not prime.\n";

    return 0;
}
MSH_CMD_EXPORT(test_future_get, test_future_get);

static int get_value() { return 10; }

int test_future_share(void)
{
    std::future<int> fut = std::async(get_value);
    std::shared_future<int> shfut = fut.share();

    // shared futures can be accessed multiple times:
    std::cout << "value: " << shfut.get() << '\n';
    std::cout << "its double: " << shfut.get() * 2 << '\n';

    return 0;
}
MSH_CMD_EXPORT(test_future_share, test_future_share);

void print_int_promise(std::future<int> &fut)
{
    int x = fut.get();
    std::cout << "value: " << x << '\n';
}

int test_future_promise(void)
{
    std::promise<int> prom; // create promise

    std::future<int> fut = prom.get_future(); // engagement with future

    std::thread th1(print_int_promise, std::ref(fut)); // send future to new thread

    prom.set_value(10); // fulfill promise
                        // (synchronizes with getting the future)
    th1.join();
    return 0;
}
MSH_CMD_EXPORT(test_future_promise, test_future_promise);

static void print_int_set_value(std::future<int> &fut)
{
    int x = fut.get();
    std::cout << "value: " << x << '\n';
}

int test_future_promise_set_value(void)
{
    std::promise<int> prom; // create promise

    std::future<int> fut = prom.get_future(); // engagement with future

    std::thread th1(print_int_set_value, std::ref(fut)); // send future to new thread

    prom.set_value(10); // fulfill promise
                        // (synchronizes with getting the future)
    th1.join();
    return 0;
}
MSH_CMD_EXPORT(test_future_promise_set_value, test_future_promise_set_value);


void pause_thread(int n) 
{
    std::this_thread::sleep_for (std::chrono::seconds(n));
    std::cout << "pause of " << n << " seconds ended\n";
}
 
int cpp_thread_test(void) 
{
    std::cout << "Spawning 3 threads...\n";
    std::thread t1 (pause_thread,1);
    std::thread t2 (pause_thread,2);
    std::thread t3 (pause_thread,3);
    std::cout << "Done spawning threads. Now waiting for them to join:\n";
    t1.join();
    t2.join();
    t3.join();
    std::cout << "All threads joined!\n";

    return 0;
}
MSH_CMD_EXPORT(cpp_thread_test, cpp_thread_test);
