/**文件说明: 官方文档学习--内核/线程间通信/信号
 * Date           Author       Notes
 * 2023-05-19     vdadh        我们
 */

#include <rtthread.h>
#include <RTSignal.h>

#define THREAD_PRIORITY     25
#define THREAD_STACK_SIZE   512
#define THREAD_TIMESLICE    5

static rt_thread_t thread1 = RT_NULL;

/*线程的信号处理函数*/
void thread1_signal_handler(int sig)
{
    //打印接收到的信号信息
    rt_kprintf("thread1 received signal %d\n", sig);
}

/*线程的入口函数*/
static void thread1_entry(void *parameter)
{
    int cnt = 0;

    rt_signal_install(SIGUSR1, thread1_signal_handler); //安装信号, 第二个参数: 指向当信号发生时用户自定义的处理函数, 由该函数来处理
    rt_signal_unmask(SIGUSR1);  //解除信号的阻塞

    while(cnt < 10)
    {
        rt_kprintf("thread1 count:%d\n", cnt);
        cnt++;
        rt_thread_mdelay(100);
    }
}

int signal_sample(void)
{
    thread1 = rt_thread_create("thread1", thread1_entry, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if(thread1 != RT_NULL) {
        rt_thread_startup(thread1);
    }

    rt_thread_mdelay(300);

    /*发送信号SIGUSR1给线程1*/
    rt_thread_kill(thread1, SIGUSR1);
    return 0;
}

MSH_CMD_EXPORT(signal_sample, signal sample);
