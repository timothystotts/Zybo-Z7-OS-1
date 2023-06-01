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

// PWM access definitions
#define PWM_AXI_CTRL_REG_OFFSET 0
#define PWM_AXI_PERIOD_REG_OFFSET 8
#define PWM_AXI_DUTY_REG_OFFSET 64

#define PWM_PERIOD_TEN_MILLISECOND ((uint32_t)1000000)
#define PWM_DUTY_CYCLE_NINE_MILLISECOND ((uint32_t)1000000 * 9 / 10)
#define PWM_DUTY_CYCLE_EIGHT_MILLISECOND ((uint32_t)1000000 * 8 / 10)
#define PWM_DUTY_CYCLE_SEVEN_MILLISECOND ((uint32_t)1000000 * 7 / 10)
#define PWM_DUTY_CYCLE_FIVE_MILLISECOND ((uint32_t)1000000 * 5 / 10)

typedef struct COLOR_PWM_TAG {
    uint32_t pwmPeriod;
} t_color_pwm_constants;

typedef struct COLOR_LED_TAG {
    uint32_t pwmIndex;
    uint32_t maxDutyCycle;
    char filamentColor;
    uint8_t silkLedIndex;
} t_color_led_constants;

typedef struct RGB_LED_TAG {
    uint8_t paletteRed;
    uint8_t paletteGreen;
    uint8_t paletteBlue;
} t_rgb_led_palette;

typedef struct RGB_LED_SILK_TAG {
    t_rgb_led_palette rgb;
    uint8_t ledSilk;
} t_rgb_led_palette_silk;

#define N_COLOR_PWMS ((int) 1)
#define N_COLOR_LEDS ((int) 6)

static uint32_t* PWM_MMAP_UIO_Address = 0;

static const t_color_pwm_constants c_color_pwms[N_COLOR_PWMS] = {
    {PWM_PERIOD_TEN_MILLISECOND}
};

static const t_color_led_constants c_color_leds[N_COLOR_LEDS] = {
        {2, PWM_DUTY_CYCLE_FIVE_MILLISECOND, 'r', 5},
        {1, PWM_DUTY_CYCLE_FIVE_MILLISECOND, 'g', 5},
        {0, PWM_DUTY_CYCLE_FIVE_MILLISECOND, 'b', 5},
        {5, PWM_DUTY_CYCLE_FIVE_MILLISECOND, 'r', 6},
        {4, PWM_DUTY_CYCLE_FIVE_MILLISECOND, 'g', 6},
        {3, PWM_DUTY_CYCLE_FIVE_MILLISECOND, 'b', 6}
};

// UIO count - for use by this program
#define UIO_PWM_CNT 1
#define UIO_GPIO_CNT 3

struct uio_access_pwm_t
{
    const char* name;
    uint32_t addr;
    int uio_num;
    int map_idx;
    uint32_t* reg_space;
    int fd;
    struct uio_info_t* info;
    int port_cnt;
};

struct uio_access_pwm_t pwmacc[UIO_PWM_CNT] =
{
    {
        "rgb",
        0x43C30000,
        -1,
        -1,
        0,
        -1,
        0,
        6
    }
};

struct uio_access_gpio_t
{
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

struct uio_access_gpio_t gpioacc[UIO_GPIO_CNT] =
{
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

void PWM_Set_Mmap_UIO_Address(uint32_t* reg_space)
{
    PWM_MMAP_UIO_Address = reg_space;
}

void PWM_Set_Period(uint32_t* reg_space, uint32_t clocks)
{
    *(reg_space + PWM_AXI_PERIOD_REG_OFFSET / 4) = clocks;
}

void PWM_Set_Duty(uint32_t* reg_space, uint32_t clocks, uint32_t pwmIndex)
{
    *(reg_space + PWM_AXI_DUTY_REG_OFFSET / 4 + pwmIndex) = clocks;
}

uint32_t PWM_Get_Period(uint32_t* reg_space)
{
    return *(reg_space + PWM_AXI_PERIOD_REG_OFFSET / 4);
}

uint32_t PWM_Get_Duty(uint32_t* reg_space, uint32_t pwmIndex)
{
    return *(reg_space + PWM_AXI_DUTY_REG_OFFSET / 4 + pwmIndex);
}

void PWM_Enable(uint32_t* reg_space)
{
    *(reg_space + PWM_AXI_CTRL_REG_OFFSET / 4) = 0x00000001;
}

void PWM_Disable(uint32_t* reg_space)
{
    *(reg_space + PWM_AXI_CTRL_REG_OFFSET / 4) = 0x00000000;
}

void InitColorLedsOff(void) {
    int i = 0;

    for(i = 0; i < N_COLOR_PWMS; ++i) {
        PWM_Set_Period(PWM_MMAP_UIO_Address, c_color_pwms[i].pwmPeriod);
    }
    for(i = 0; i < N_COLOR_LEDS; ++i) {
        PWM_Set_Duty(PWM_MMAP_UIO_Address, 0, c_color_leds[i].pwmIndex);
    }
    for(i = 0; i < N_COLOR_PWMS; ++i) {
        PWM_Enable(PWM_MMAP_UIO_Address);
    }
}

int SetColorLedPercent(const uint8_t ledSilk, const char color, const uint32_t percentFixPt) {
    int i = 0;
    uint32_t dutyClocks = 0;
    int ret = 1; // Failure

    for (i = 0; (i < N_COLOR_LEDS) && ret; ++i) {
        if (c_color_leds[i].silkLedIndex == ledSilk) {
            if (c_color_leds[i].filamentColor == color) {
                dutyClocks = percentFixPt * c_color_leds[i].maxDutyCycle / 1000;
                PWM_Set_Duty(PWM_MMAP_UIO_Address, dutyClocks, c_color_leds[i].pwmIndex);
                ret = 0; // Success
            }
        }
    }

    return ret;
}

int SetRgbPaletteLed(const uint8_t ledSilk, const t_rgb_led_palette* palette) {
    int ret0 = 0;
    int ret1 = 0;
    int ret2 = 0;

    ret0 = SetColorLedPercent(ledSilk, 'r', palette->paletteRed * 1000 / 255);
    ret1 = SetColorLedPercent(ledSilk, 'g', palette->paletteGreen * 1000 / 255);
    ret2 = SetColorLedPercent(ledSilk, 'b', palette->paletteBlue * 1000 / 255);

    return (ret0 || ret1 || ret2) ? 1 : 0;
}

static int find_map_gpio_pwm(struct uio_info_t *info, int uio_num)
{
    int mi;
    int acc;

    for (mi = 0; mi < MAX_UIO_MAPS; ++mi)
    {
        for (acc = 0; acc < UIO_GPIO_CNT; ++acc)
        {
            if (info->maps[mi].addr == gpioacc[acc].addr)
            {
                gpioacc[acc].uio_num = uio_num;
                gpioacc[acc].map_idx = mi;
                gpioacc[acc].info = info;
            }
        }

        for (acc = 0; acc < UIO_PWM_CNT; ++acc)
        {
            if (info->maps[mi].addr == pwmacc[acc].addr)
            {
                pwmacc[acc].uio_num = uio_num;
                pwmacc[acc].map_idx = mi;
                pwmacc[acc].info = info;
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
    uint32_t* reg_space = gpioacc[acc_idx].reg_space;
    bool i1 = gpioacc[acc_idx].enable_ch1 && (! gpioacc[acc_idx].output_ch1) && gpioacc[acc_idx].has_irq;
    bool i2 = gpioacc[acc_idx].enable_ch2 && (! gpioacc[acc_idx].output_ch2) && gpioacc[acc_idx].has_irq;

    if (i1 && i2)
        printf("%s: Tracking interrupts for channels 1 & 2.\n",
            gpioacc[acc_idx].name);
    else if (i1)
        printf("%s: Tracking interrupts for channel 1.\n",
            gpioacc[acc_idx].name);
    else if (i2)
        printf("%s: Tracking interrupts for channel 2.\n",
            gpioacc[acc_idx].name);
    else
        printf("%s: Not tracking interrupts.\n",
            gpioacc[acc_idx].name);

    *(reg_space + GPIO_TRI / 4) = gpioacc[acc_idx].output_ch1 ? 0x00000000 : 0xFFFFFFFF;
    *(reg_space + GPIO2_TRI / 4) = gpioacc[acc_idx].output_ch2 ? 0x00000000 : 0xFFFFFFFF;
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

    info_list = uio_find_devices(-1);
    if (!info_list)
        printf("No UIO devices found.\n");

    info = info_list;

    while(info)
    {
        uio_get_all_info(info);
        uio_get_device_attributes(info);
        find_map_gpio_pwm(info, info->uio_num);

        info = info->next;
    }

    printf("Found the following GPIO:\n");
    for (int idx = 0; idx < UIO_GPIO_CNT; ++idx)
    {
        char dev_name[16];
        snprintf(dev_name,sizeof(dev_name),"/dev/uio%d",gpioacc[idx].uio_num);
        gpioacc[idx].fd = open(dev_name,O_RDWR);

        if (gpioacc[idx].fd > -1)
        {
            uio_mmap(gpioacc[idx].info, gpioacc[idx].fd);
            gpioacc[idx].reg_space = (uint32_t*)(gpioacc[idx].info->maps[gpioacc[idx].map_idx].internal_addr);
        }

        printf("name=%s addr=%08x uio=%d map=%d fd=%d rs=%p info=%p has_irq=%d\n",
            gpioacc[idx].name,
            gpioacc[idx].addr,
            gpioacc[idx].uio_num,
            gpioacc[idx].map_idx,
            gpioacc[idx].fd,
            gpioacc[idx].reg_space,
            gpioacc[idx].info,
            gpioacc[idx].has_irq);
    }

    printf("Found the following PWM:\n");
    for (int idx = 0; idx < UIO_PWM_CNT; ++idx)
    {
        char dev_name[16];
        snprintf(dev_name,sizeof(dev_name),"/dev/uio%d",pwmacc[idx].uio_num);
        pwmacc[idx].fd = open(dev_name,O_RDWR);

        if (pwmacc[idx].fd > -1)
        {
            uio_mmap(pwmacc[idx].info, pwmacc[idx].fd);
            pwmacc[idx].reg_space = (uint32_t*)(pwmacc[idx].info->maps[pwmacc[idx].map_idx].internal_addr);
        }

        printf("name=%s addr=%08x uio=%d map=%d fd=%d rs=%p info=%p port_cnt=%d\n",
            pwmacc[idx].name,
            pwmacc[idx].addr,
            pwmacc[idx].uio_num,
            pwmacc[idx].map_idx,
            pwmacc[idx].fd,
            pwmacc[idx].reg_space,
            pwmacc[idx].info,
            pwmacc[idx].port_cnt);

        if (strcmp(pwmacc[idx].name, "rgb") == 0)
        {
            PWM_Set_Mmap_UIO_Address(pwmacc[idx].reg_space);
        }
    }
}

static void handle_fd(int fd)
{
    static int total_pir_count = 0;
    static int total_sw_count = 0;
    static int total_btn_count = 0;

    static uint32_t buttons = 0;
    static uint32_t switches = 0;
    static t_rgb_led_palette_silk rgb5 = {
        {2, 2, 2}, 5
    };
    static t_rgb_led_palette_silk rgb6 = {
        {2, 2, 2}, 6
    };

    uint32_t input1;
    uint32_t input2;

    for (int idx = 0; idx < UIO_GPIO_CNT; ++idx)
    {
        if (gpioacc[idx].fd == fd)
        {
            if (strcmp(gpioacc[idx].name, "2pir") == 0)
            {
                input1 = (*(gpioacc[idx].reg_space + GPIO_DATA / 4));
                reinit_gpio_interrupt(gpioacc[idx].reg_space);
                write_gpio_uiofd(fd);
                total_pir_count += 1;

                printf("PIR2 detection: val=%01x,cnt=%ld\n", input1, total_pir_count);

                *(gpioacc[1].reg_space + GPIO_DATA / 4) ^= 0xC;

                if (buttons)
                {
                    rgb5.rgb.paletteBlue = 2;
                    rgb5.rgb.paletteGreen = 2;
                    rgb5.rgb.paletteRed = 2;
                    rgb6.rgb.paletteBlue = 2;
                    rgb6.rgb.paletteGreen = 2;
                    rgb6.rgb.paletteRed = 2;
                }

                if (switches == 0x1)
                {
                    rgb5.rgb.paletteRed = rgb5.rgb.paletteRed + 32;
                    rgb6.rgb.paletteBlue = rgb6.rgb.paletteBlue - 32;
                    SetRgbPaletteLed(rgb5.ledSilk, &rgb5);
                    SetRgbPaletteLed(rgb6.ledSilk, &rgb6);
                }
                else if (switches == 0x2)
                {
                    rgb5.rgb.paletteBlue = rgb5.rgb.paletteBlue + 32;
                    rgb6.rgb.paletteGreen = rgb6.rgb.paletteGreen - 32;
                    SetRgbPaletteLed(rgb5.ledSilk, &rgb5);
                    SetRgbPaletteLed(rgb6.ledSilk, &rgb6);    
                }
                else if (switches == 0x4)
                {
                    rgb5.rgb.paletteGreen = rgb5.rgb.paletteGreen + 32;
                    rgb6.rgb.paletteRed = rgb6.rgb.paletteRed - 32;
                    SetRgbPaletteLed(rgb5.ledSilk, &rgb5);
                    SetRgbPaletteLed(rgb6.ledSilk, &rgb6);    
                }
                else if (switches == 0x8)
                {
                    rgb5.rgb.paletteRed = 1;
                    rgb5.rgb.paletteGreen = 1;
                    rgb5.rgb.paletteBlue = rgb5.rgb.paletteBlue + 4;
                    rgb6.rgb.paletteRed = 1;
                    rgb6.rgb.paletteGreen = 1;
                    rgb6.rgb.paletteBlue = rgb5.rgb.paletteBlue + 4;
                    SetRgbPaletteLed(rgb5.ledSilk, &rgb5);
                    SetRgbPaletteLed(rgb6.ledSilk, &rgb6);    
                }
            }
            else if (strcmp(gpioacc[idx].name, "sw_btn") == 0)
            {
                // Switches
                input1 = (*(gpioacc[idx].reg_space + GPIO_DATA / 4));
                input2 = (*(gpioacc[idx].reg_space + GPIO2_DATA / 4));
                reinit_gpio_interrupt(gpioacc[idx].reg_space);
                write_gpio_uiofd(fd);

                if (input1 != switches)
                {
                    switches = input1;
                    total_sw_count += 1;
                    printf("SWS  detection: val=%01x,cnt=%ld\n", input1, total_sw_count);

                    *(gpioacc[1].reg_space + GPIO_DATA / 4) ^= 0x1;
                }

                if (input2 != buttons)
                {
                    buttons = input2;
                    total_btn_count += 1;
                    printf("BTNS detection: val=%01x,cnt=%ld\n", input2, total_btn_count);

                    *(gpioacc[1].reg_space + GPIO_DATA / 4) ^= 0x2;
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
        if (gpioacc[idx].fd > -1 && gpioacc[idx].has_irq)
        {
            printf("Setting select for FD=%d\n", gpioacc[idx].fd);
            safe_fd_set(gpioacc[idx].fd, &master, &max_fd);

            reinit_gpio_interrupt(gpioacc[idx].reg_space);
            write_gpio_uiofd(gpioacc[idx].fd);
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
    InitColorLedsOff();
    process_gpio();

    printf("Quitting program.\n");

    for (int idx = 0; idx < UIO_GPIO_CNT; ++idx)
    {
        if (gpioacc[idx].fd > -1)
        {
            close(gpioacc[idx].fd);
            uio_munmap(gpioacc[idx].info);
        }
    }

    uio_free_info(info_list);
    exit(0);
}
