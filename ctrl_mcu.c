/**
 * @file ctrl_mcu.c
 * @brief
 * @details
 * @mainpage
 */
#include "ctrl_mcu.h"
#include "ctrl.h"

#define UART2MCU_FILE_NAME "/dev/ttyAMA2"
// #define UART2MCU_FILE_NAME "/dev/ttyS2"

int fd_uart = 0;

pthread_t pid_uart1_recv = 0;
void* mcu_uart_recv_thread(void* para);

static inline void mcu_cmd_classify(const uint8_t* cmd, int size);
static inline void mcu_status_handler(const uint8_t* cmd, int size);
static inline void motor_status_handler(const uint8_t* cmd, int size);
extern void ir_cmd_handler(const uint8_t* cmd, int size);

/**
 * @brief 串口模式设置
 */
int mcu_uart_init(void)
{
    struct termios term; /* Create the structure                          */

    if (fd_uart)
    {
        // LOG_WARN("%s has been opened! close it and reopen now!", UART2MCU_FILE_NAME);
        // close(fd_uart);
        // fd_uart = 0;
        return 0;
    }

    if (fd_uart = open(UART2MCU_FILE_NAME, O_RDWR | O_NOCTTY | O_NDELAY), fd_uart <= 0)
    {
        LOG_ERROR("error! open %s failure!", UART2MCU_FILE_NAME);
        return -1;
    }

    tcgetattr(fd_uart, &term); /* Get the current attributes of the Serial port */

    /* Setting the Baud rate */
    cfsetispeed(&term, B460800);
    cfsetospeed(&term, B460800);

    /* 8N1 Mode */
    term.c_cflag = CS8 | CREAD | CLOCAL | B460800;
    term.c_iflag = 0;
    term.c_oflag = 0;
    term.c_lflag = 0;

    /* Setting Time outs */
    term.c_cc[VMIN] = 0;  /* 至少读出的字节数 */
    term.c_cc[VTIME] = 0; /* 等待时间,单位100ms */

    tcsetattr(fd_uart, TCIOFLUSH, &term);
    // tcsetattr(fd_uart, TCIFLUSH, &term);

    cfgetospeed(&term);

    if (pthread_create(&pid_uart1_recv, NULL, mcu_uart_recv_thread, NULL))
    {
        LOG_ERROR("mcu uart recv thread init failed whith %s", strerror(errno));
        return -1;
    }

    return 0;
}

/**
 * @brief 清空串口缓存
 */
void mcu_clear_input_buffer(void)
{
    tcflush(fd_uart, TCIFLUSH);  //清空输入缓存
    // tcflush(fd_uart, TCOFLUSH); //清空输出缓存
    // tcflush(fd, TCIOFLUSH); //清空输入输出缓存
    // tcsetattr(m_ifd, TCSANOW, &strctNewTermios) ；
}

/**
 * @brief UART2MCU发送数据
 */
int mcu_send(const uint8_t* data, int size)
{
    // byte_cmd_dump("send cmd", data, size, 16);
    if (fd_uart)
    {
        return write(fd_uart, data, size);
    }
    else
    {
        LOG_ERROR("mcu uart is not initialized!");
        return -1;
    }
}

/**
 * @brief UART2MCU接收线程
 */
void* mcu_uart_recv_thread(void* para)
{
#define READ_BUFFER_SIZE 256
#define CMD_BUFFER_SIZE 128
    static uint8_t read_buffer[READ_BUFFER_SIZE] = {0};
    static uint8_t cmd_buffer[CMD_BUFFER_SIZE] = {0};

    fd_set rd;
    int nread;

    static int cmd_flag = -1;
    static int start_flag = 0;
    static int cmd_index = 0;

    while (1)
    {
        FD_ZERO(&rd);
        FD_SET(fd_uart, &rd);

        if (select(fd_uart + 1, &rd, NULL, NULL, NULL) < 0)
        {
            LOG_ERROR("Uart receive select error!");
        }

        if (FD_ISSET(fd_uart, &rd))
        {
            nread = read(fd_uart, read_buffer, sizeof(read_buffer));
            if (nread <= 0)
                continue;

            // byte_cmd_dump("recv data:", read_buffer, nread, 16);

            for (int i = 0; i < nread; i++)
            {
                switch (cmd_flag)
                {
                    case -1: {
                        if (read_buffer[i] == CMD_FLAG)
                            cmd_flag = 0;
                    }
                    break;

                    case 0: {
                        switch (read_buffer[i])
                        {
                            case CMD_START1:
                                cmd_flag = 1;
                                // LOG_INFO("got start flag 1");
                                break;
                            case CMD_END1:
                                cmd_flag = 10;
                                // LOG_INFO("got end flag 1");
                                break;
                            case CMD_FLAG: cmd_flag = 0; break;
                            default: cmd_flag = -1;
                        }
                    }
                    break;
                    case 1: {
                        if (read_buffer[i] == CMD_START2)
                        {
                            // LOG_INFO("got start flag 2");
                            cmd_flag = -1;
                            start_flag = 1;
                            cmd_index = 0;
                            continue;
                        }
                        cmd_flag = -1;
                    }
                    break;
                    case 10: {
                        if (read_buffer[i] == CMD_END2)
                        {
                            // LOG_INFO("got end flag 2");
                            if (start_flag)
                                mcu_cmd_classify(cmd_buffer, cmd_index - 2);
                            start_flag = 0;
                            cmd_index = 0;
                        }
                        cmd_flag = -1;
                    }
                    break;
                }

                cmd_buffer[cmd_index++] = read_buffer[i];
                if (cmd_index >= CMD_BUFFER_SIZE)
                {
                    // LOG_INFO("cmd_buffer overflow");
                    cmd_index = 0;
                    start_flag = 0;
                    cmd_flag = -1;
                }
            }
        }
    }
    return NULL;
}

/**
 * @brief MCU初始化
 */
int ctrl_mcu_init(void)
{
    if (mcu_uart_init())
    {
        return -1;
    }

    return 0;
}
