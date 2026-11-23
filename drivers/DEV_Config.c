/*****************************************************************************
* | File        :   DEV_Config.c
* | Author      :   Waveshare team (modified)
* | Info        :   GPIO via bcm2835, SPI via kernel spidev (transaction API)
******************************************************************************/
#include "DEV_Config.h"
#include <fcntl.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <bcm2835.h>

/* --------------------------------------------------------------------------
 * Pin mapping (BCM numbering) - Waveshare High-Precision AD HAT defaults.
 * ------------------------------------------------------------------------ */
int DEV_RST_PIN  = 18;  /* Pin 12 */
int DEV_CS_PIN   = 22;  /* Pin 15 */
int DEV_DRDY_PIN = 17;  /* Pin 11 */

static int spi_fd = -1;
static pthread_mutex_t spi_lock = PTHREAD_MUTEX_INITIALIZER;

void DEV_Digital_Write(UWORD Pin, UBYTE Value) { bcm2835_gpio_write(Pin, Value); }
UBYTE DEV_Digital_Read(UWORD Pin)               { return bcm2835_gpio_lev(Pin); }
void DEV_Delay_ms(UDOUBLE xms)                  { bcm2835_delay(xms); }

uint64_t DEV_Millis(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)(ts.tv_nsec / 1000000);
}

int DEV_SPI_Transfer(const UBYTE *tx, UBYTE *rx, size_t len)
{
    if (spi_fd < 0 || tx == NULL || len == 0) return -1;

    struct spi_ioc_transfer tr;
    memset(&tr, 0, sizeof(tr));
    tr.tx_buf        = (unsigned long)tx;
    tr.rx_buf        = (unsigned long)rx;   /* 0 (NULL) is allowed */
    tr.len           = (uint32_t)len;
    tr.speed_hz      = ADC_SPI_SPEED_HZ;
    tr.bits_per_word = 8;

    pthread_mutex_lock(&spi_lock);
#if !ADC_KERNEL_CS
    bcm2835_gpio_write(DEV_CS_PIN, LOW);
#endif
    /* One message = the kernel will not schedule another device's transfer
     * on this bus until it completes. */
    int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
#if !ADC_KERNEL_CS
    bcm2835_gpio_write(DEV_CS_PIN, HIGH);
#endif
    pthread_mutex_unlock(&spi_lock);

    if (ret < (int)len) {
        perror("SPI transfer failed");
        return -1;
    }
    return 0;
}

static void DEV_GPIO_Init(void)
{
#if !ADC_KERNEL_CS
    bcm2835_gpio_fsel(DEV_CS_PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(DEV_CS_PIN, HIGH);
#endif
    bcm2835_gpio_fsel(DEV_RST_PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(DEV_RST_PIN, HIGH);

    bcm2835_gpio_fsel(DEV_DRDY_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(DEV_DRDY_PIN, BCM2835_GPIO_PUD_UP);
}

UBYTE DEV_Module_Init(void)
{
    if (!bcm2835_init()) {
        fprintf(stderr, "bcm2835 init failed (run with sudo / check /dev/gpiomem)\n");
        return 1;
    }
    DEV_GPIO_Init();

    spi_fd = open(ADC_SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        fprintf(stderr, "Failed to open SPI device %s: %s\n", ADC_SPI_DEVICE, strerror(errno));
        fprintf(stderr, "Check 'dtparam=spi=on' and the overlays in /boot/firmware/config.txt\n");
        bcm2835_close();
        return 1;
    }

#if ADC_KERNEL_CS
    uint8_t mode = SPI_MODE_1;
#else
    uint8_t mode = SPI_MODE_1 | SPI_NO_CS;
#endif
    uint32_t speed = ADC_SPI_SPEED_HZ;
    uint8_t  bits  = 8;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("Failed to configure SPI");
        close(spi_fd);
        spi_fd = -1;
        bcm2835_close();
        return 1;
    }

    DEV_Delay_ms(100);
    printf("ADC SPI init OK (%s, CS: %s)\n", ADC_SPI_DEVICE,
           ADC_KERNEL_CS ? "kernel" : "GPIO22 (user space)");
    return 0;
}

void DEV_Module_Exit(void)
{
    DEV_Digital_Write(DEV_RST_PIN, 0);
#if !ADC_KERNEL_CS
    DEV_Digital_Write(DEV_CS_PIN, 1);
#endif
    if (spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }
    bcm2835_close();
}
