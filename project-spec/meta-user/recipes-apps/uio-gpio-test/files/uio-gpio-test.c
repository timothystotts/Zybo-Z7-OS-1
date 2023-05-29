/*
 * This code can be freely used and copied for your own purposes.
 * It is provided as is, no warranty provided.
 */

/*
 * References:
 *
 * https://www.gnu.org/software/libc/manual/html_node/Sigaction-Function-Example.html
 * https://stackoverflow.com/questions/38304715/using-select-to-detect-a-block-on-a-uio-device-file
 * http://jhshi.me/2013/11/02/use-select-to-monitor-multiple-file-descriptors/index.html
 * https://www.osadl.org/UIO.uio0.0.html
 * https://forum.digilent.com/topic/16550-uio-access-including-interrupts-when-using-petalinux/
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <unistd.h>
#include <string.h>
#include <sys/select.h>
#include <assert.h>
#include <signal.h>
#include <uio_helper.h>

// GPIO access definitions
#define GPIO_DATA           0x000
#define GPIO_TRI            0x004
#define GPIO2_DATA          0x008
#define GPIO2_TRI           0x00C
#define GIER                0x11C
#define IER                 0x128
#define ISR                 0x120

#define GIEn                0x80000000

#define UIO_GPIO_CNT 3

struct uio_access_t {
    const char* name;
    uint32_t addr;
    int uio_num;
    int map_idx;
    uint32_t* reg_space;
    int fd;
    struct uio_info_t* info;
    bool enable_ch1;
    bool enable_ch2;
    bool output_ch1;
    bool output_ch2;
    bool has_irq;
};

struct uio_access_t uioacc[UIO_GPIO_CNT] = {
    {   "sw_btn",
        0x41210000,
        -1,
        -1,
        0,
        -1,
        0,
        true, true, false, false, true },
    {   "led",
        0x41220000,
        -1,
        -1,
        0,
        -1,
        0,
        true, false, true, false, false },
    {   "2pir",
        0x41240000,
        -1,
        -1,
        0,
        -1,
        0,
        true, false, false, false, true },

};

static struct uio_info_t *info_list;
static int max_fd = 0;
static bool global_quit = false;

/* add a fd to fd_set, and update max_fd */
static int
safe_fd_set(int fd, fd_set* fds, int* max_fd) {
    assert(max_fd != NULL);

    FD_SET(fd, fds);
    if (fd > *max_fd) {
        *max_fd = fd;
    }
    return 0;
}

/* clear fd from fds, update max fd if needed */
static int
safe_fd_clr(int fd, fd_set* fds, int* max_fd) {
    assert(max_fd != NULL);

    FD_CLR(fd, fds);
    if (fd == *max_fd) {
        (*max_fd)--;
    }
    return 0;
}

static int find_map(struct uio_info_t *info, int uio_num)
{
    int mi;
    int acc;

    for (mi = 0; mi < MAX_UIO_MAPS; ++mi)
    {
        for (acc = 0; acc < UIO_GPIO_CNT; ++acc)
        {
            if (info->maps[mi].addr == uioacc[acc].addr)
            {
                uioacc[acc].uio_num = uio_num;
                uioacc[acc].map_idx = mi;
                uioacc[acc].info = info;
            }
        }
    }
}

static void write_gpio_uiofd(int fd)
{
    char dummy_buf[4];

    memset(dummy_buf, 0x00, sizeof(dummy_buf));
    dummy_buf[0] = 1;
    (void)write(fd, dummy_buf, 4);
    usleep(1);
}

static void firstinit_gpio_interrupt(int acc_idx)
{
    uint32_t* reg_space = uioacc[acc_idx].reg_space;
    bool i1 = uioacc[acc_idx].enable_ch1 && (! uioacc[acc_idx].output_ch1) && uioacc[acc_idx].has_irq;
    bool i2 = uioacc[acc_idx].enable_ch2 && (! uioacc[acc_idx].output_ch2) && uioacc[acc_idx].has_irq;

    if (i1 && i2)
        printf("%s: Tracking interrupts for channels 1 & 2.\n",
            uioacc[acc_idx].name);
    else if (i1)
        printf("%s: Tracking interrupts for channel 1.\n",
            uioacc[acc_idx].name);
    else if (i2)
        printf("%s: Tracking interrupts for channel 2.\n",
            uioacc[acc_idx].name);
    else
        printf("%s: Not tracking interrupts.\n",
            uioacc[acc_idx].name);

    *(reg_space + GPIO_TRI / 4) = uioacc[acc_idx].output_ch1 ? 0x00000000 : 0xFFFFFFFF;
    *(reg_space + GPIO2_TRI / 4) = uioacc[acc_idx].output_ch2 ? 0x00000000 : 0xFFFFFFFF;
    *(reg_space + IER / 4) = (i1 && i2) ? 0x3 : (i1 ? 0x1 : (i2 ? 0x2 : 0x00));
    *(reg_space + GIER / 4) = (i1 || i2) ? GIEn : 0x00000000;
}

static void reinit_gpio_interrupt(uint32_t* reg_space)
{
    ssize_t dummy_rd;

    dummy_rd = *(reg_space + ISR / 4);
    usleep(1);
    *(reg_space + ISR / 4) = dummy_rd;
    usleep(1);
}

static void detect_uio()
{
    struct uio_info_t *info;

    info_list = uio_find_devices_byname("gpio");
    if (!info_list)
        printf("No GPIO UIO devices found.\n");

    info = info_list;

    while(info)
    {
        uio_get_all_info(info);
        uio_get_device_attributes(info);
        find_map(info, info->uio_num);

        info = info->next;
    }

    printf("Found the following GPIO:\n");
    for (int idx = 0; idx < UIO_GPIO_CNT; ++idx)
    {
        char dev_name[16];
        snprintf(dev_name,sizeof(dev_name),"/dev/uio%d",uioacc[idx].uio_num);
        uioacc[idx].fd = open(dev_name,O_RDWR);

        if (uioacc[idx].fd > -1)
        {
            uio_mmap(uioacc[idx].info, uioacc[idx].fd);
            uioacc[idx].reg_space = (uint32_t*)(uioacc[idx].info->maps[uioacc[idx].map_idx].internal_addr);
        }

        printf("name=%s addr=%08x uio=%d map=%d fd=%d rs=%p info=%p\n",
            uioacc[idx].name,
            uioacc[idx].addr,
            uioacc[idx].uio_num,
            uioacc[idx].map_idx,
            uioacc[idx].fd,
            uioacc[idx].reg_space,
            uioacc[idx].info);
    }
}

static void handle_fd(int fd)
{
    static int total_pir_count = 0;
    static int total_sw_count = 0;
    static int total_btn_count = 0;

    static uint32_t buttons = 0;
    static uint32_t switches = 0;

    uint32_t input1;
    uint32_t input2;

    for (int idx = 0; idx < UIO_GPIO_CNT; ++idx)
    {
        if (uioacc[idx].fd == fd)
        {
            if (strcmp(uioacc[idx].name, "2pir") == 0)
            {
                input1 = (*(uioacc[idx].reg_space + GPIO_DATA / 4));
                reinit_gpio_interrupt(uioacc[idx].reg_space);
                write_gpio_uiofd(fd);
                total_pir_count += 1;

                printf("PIR2 detection: val=%01x,cnt=%ld\n", input1, total_pir_count);

                *(uioacc[1].reg_space + GPIO_DATA / 4) ^= 0xC;
            }
            else if (strcmp(uioacc[idx].name, "sw_btn") == 0)
            {
                // Switches
                input1 = (*(uioacc[idx].reg_space + GPIO_DATA / 4));
                input2 = (*(uioacc[idx].reg_space + GPIO2_DATA / 4));
                reinit_gpio_interrupt(uioacc[idx].reg_space);
                write_gpio_uiofd(fd);

                if (input1 != switches)
                {
                    switches = input1;
                    total_sw_count += 1;
                    printf("SWS  detection: val=%01x,cnt=%ld\n", input1, total_sw_count);

                    *(uioacc[1].reg_space + GPIO_DATA / 4) ^= 0x1;
                }

                if (input2 != buttons)
                {
                    buttons = input2;
                    total_btn_count += 1;
                    printf("BTNS detection: val=%01x,cnt=%ld\n", input2, total_btn_count);

                    *(uioacc[1].reg_space + GPIO_DATA / 4) ^= 0x2;
                }
            }
        }
    }
}

static void process_gpio()
{
    fd_set master;
    int evt_count = 0;

    /* add stdin and the sock fd to master fd_set */
    FD_ZERO(&master);

    for (int idx = 0; idx < UIO_GPIO_CNT; ++idx)
    {
        if (uioacc[idx].fd > -1 && uioacc[idx].has_irq)
        {
            printf("Setting select for FD=%d\n", uioacc[idx].fd);
            safe_fd_set(uioacc[idx].fd, &master, &max_fd);

            reinit_gpio_interrupt(uioacc[idx].reg_space);
            write_gpio_uiofd(uioacc[idx].fd);
        }
    }

    while(! global_quit)
    {
        /* back up master */
        fd_set dup = master;

        /* note the max_fd+1 */
        if (select(max_fd+1, &dup, NULL, NULL, NULL) < 0)
        {
            perror("select");
        }

        if (global_quit)
            break;

        /* check which fd is avaialbe for read */
        for (int fd = 0; fd <= max_fd; fd++)
        {
            if (FD_ISSET(fd, &dup))
            {
                (void)read(fd, &evt_count, 4);
                usleep(1);
                handle_fd(fd);
            }
        }

        usleep(100);
    }
}

static void termination_handler (int signum)
{
    global_quit = true;
}

int main(int argc, char** argv)
{
    struct sigaction new_action, old_action;

    new_action.sa_handler = termination_handler;
    sigemptyset (&new_action.sa_mask);
    new_action.sa_flags = 0;

    sigaction (SIGINT, NULL, &old_action);
    if (old_action.sa_handler != SIG_IGN)
        sigaction (SIGINT, &new_action, NULL);

    sigaction (SIGHUP, NULL, &old_action);
    if (old_action.sa_handler != SIG_IGN)
        sigaction (SIGHUP, &new_action, NULL);

    sigaction (SIGTERM, NULL, &old_action);
    if (old_action.sa_handler != SIG_IGN)
        sigaction (SIGTERM, &new_action, NULL);

    detect_uio();
    for (int idx = 0; idx < UIO_GPIO_CNT; ++idx)
    {
        firstinit_gpio_interrupt(idx);
    }
    process_gpio();

    printf("Quitting program.\n");

    for (int idx = 0; idx < UIO_GPIO_CNT; ++idx)
    {
        if (uioacc[idx].fd > -1)
        {
            close(uioacc[idx].fd);
            uio_munmap(uioacc[idx].info);
        }
    }

    uio_free_info(info_list);
    exit(0);
}
