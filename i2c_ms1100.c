/**文件说明:
 * Date           Author       Notes
 * 2023-05-31     vdadh        RT-Thread学习--设备和驱动/I2C总线设备
 * -驱动教学霍尔前端板上的MS1100数模转换器
 * -博客参考: https://www.jianshu.com/p/e0b448995316?from=singlemessage
 */
#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"

//#define LED_PIN_NUM 66  //PE2
#define LED_PIN_NUM GET_PIN(E, 12)

#define THREAD_STACK_SIZE   512
#define THREAD_PRIORITY     10
#define THREAD_TIMESLICE    5

#define I2C_BUS_NAME        "i2c1"  //传感器连接的I2C总线设备名称
#define MS1100_ADDR         0x90    //从机地址--读1写0

static struct rt_i2c_bus_device *i2c_bus = RT_NULL; //I2C总线设备句柄
static rt_bool_t initialized = RT_FALSE; //传感器初始状态
static rt_thread_t ms1100_thread = RT_NULL;
static rt_uint8_t config_set;

/*-------------------------- I2C Write Read --------------------------*/
//写传感器寄存器
static rt_err_t write_reg(struct rt_i2c_bus_device *bus, rt_uint8_t data)
{
    rt_uint8_t buf[1];
    struct rt_i2c_msg msgs;

    buf[0] = data;
    msgs.addr = MS1100_ADDR;    //从机地址
    msgs.flags = RT_I2C_WR;     //写0
    msgs.buf = buf;
    msgs.len = 1;

    //调用I2C设备接口传输数据--返回传输字节数--表明传输成功
    if(rt_i2c_transfer(bus, &msgs, 1) == 1) {
        rt_kprintf("i2c_write success\n");
        return RT_EOK;
    } else {
        rt_kprintf("i2c_write failed\n");
        return -RT_ERROR;
    }
}

//读传感器寄存器
static rt_err_t read_regs(struct rt_i2c_bus_device *bus, rt_uint8_t *buf, rt_uint8_t len)
{
    struct rt_i2c_msg msgs;

    msgs.addr = MS1100_ADDR;
    msgs.flags = RT_I2C_RD; //读1
    msgs.buf = buf;
    msgs.len = len;

    //调用I2C设备接口传输数据
    if(rt_i2c_transfer(bus, &msgs, 1) == 1) {
        rt_kprintf("i2c_read success\n");
        return RT_EOK;  //0
    } else {
        rt_kprintf("i2c_read failed\n");
        return -RT_ERROR;   //-1
    }
}

/*-------------------------- MS1100 --------------------------*/
static void ms1100_init()
{
    i2c_bus = (struct rt_i2c_bus_device *)rt_device_find(I2C_BUS_NAME); //查找I2C总线设备，获取I2C总线设备句柄
    if(i2c_bus == RT_NULL) {
        rt_kprintf("can't find %s device!\n", I2C_BUS_NAME);
    }
    else {
        rt_kprintf("%s device find success\n", I2C_BUS_NAME);
        write_reg(i2c_bus, config_set);
        rt_thread_mdelay(400);
        initialized = RT_TRUE;  //初始化标志位
    }
}

static void read_val(double *val)
{
    rt_uint8_t rx_data[3];
    int16_t data;
    float dr = 32768.0;
    int pga = 1;

    read_regs(i2c_bus, rx_data, 3);
    data = rx_data[0]*256 + rx_data[1];
    rt_kprintf("output num is %d\n", data);

    switch(0x0C & config_set)
    {
        case 0x00:
            dr = 2048.0;
            rt_kprintf("dr = 00\n");
            break;
        case 0x04:
            dr = 8192.0;
            rt_kprintf("dr = 01\n");
            break;
        case 0x08:
            dr = 16384.0;
            rt_kprintf("dr = 10\n");
            break;
        case 0x0C:
            dr = 32768.0;
            rt_kprintf("dr = 11\n");
            break;
        default:
            dr = 32768.0;
            break;
    }

    switch(0x03 & config_set)
    {
        case 0x00:
            dr = 1;
            rt_kprintf("pga = 00\n");
            break;
        case 0x01:
            dr = 2;
            rt_kprintf("pga = 01\n");
            break;
        case 0x02:
            dr = 4;
            rt_kprintf("pga = 10\n");
            break;
        case 0x03:
            dr = 8;
            rt_kprintf("pga = 11\n");
            break;
        default:
            dr = 1;
            break;
    }

    *val = ((double)data*2.048) / (dr*pga);
}

/*-------------------------- Thread Entry --------------------------*/
static void ms1100_thread_entry(void *parameter)
{
    double val = 0.0;
    rt_uint8_t led_status;

    if(!initialized) {
        ms1100_init();
    }
    if(initialized) {
        while(1) {
            led_status = rt_pin_read(LED_PIN_NUM);
            rt_pin_write(LED_PIN_NUM, led_status^1);

            rt_kprintf("ms1100 init success\n");
            read_val(&val);
            rt_kprintf("the ms1100 val is %d.%d\n", (int)val, (int)(val*10)%10);

            rt_thread_mdelay(2000);
        }
    }
    else {
        rt_kprintf("initialize sensor failed!\n");
    }
}

int ms1100(int argc, char *argv[])
{
    char argv_get[RT_NAME_MAX]; //8
    rt_uint8_t high_byte_uint, low_byte_uint;

    //获取终端输出字符
    if(argc == 2) {
        rt_strncpy(argv_get, argv[1], RT_NAME_MAX); //字符串复制
    } else {
        rt_strncpy(argv_get, "8C", RT_NAME_MAX);    //用户没有输入，就用默认
    }
    rt_kprintf("argv get %s\n", argv_get);
    high_byte_uint = (rt_uint8_t)argv_get[0];
    low_byte_uint = (rt_uint8_t)argv_get[1];

    //强制--ASCII转换
    if(isdigit(high_byte_uint)) {
        high_byte_uint = high_byte_uint - '0';      //如果是数字，返回值为字符值减去 '0' 的 ASCII 码值。
    } else if(isalpha(high_byte_uint)) {
        high_byte_uint = high_byte_uint - 'A' + 10; //如果是字母，返回值应该是 A-F 对应的数值加上 10。
    }
    if(isdigit(low_byte_uint)) {
        low_byte_uint = low_byte_uint - '0';
    } else if(isalpha(low_byte_uint)) {
        low_byte_uint = low_byte_uint - 'A' + 10;
    }
    config_set = (high_byte_uint << 8) | low_byte_uint;

    rt_pin_mode(LED_PIN_NUM, PIN_MODE_OUTPUT);  //LED设置为输出模式

//    rt_i2c_bus_device_register();

    ms1100_thread = rt_thread_create("ms1100_thread", ms1100_thread_entry, RT_NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);
    if(ms1100_thread != RT_NULL) {
        rt_thread_startup(ms1100_thread);
    }
    return 0;
}

MSH_CMD_EXPORT(ms1100, i2c ms1100 sample);
