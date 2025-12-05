/*****************************************************************************
* | File        :   DEV_Config.c
* | Author      :   Waveshare team (Modified for Thread-Safe & Kernel Compat)
* | Info        :   Hybrid Driver: GPIO via bcm2835, SPI via spidev0.1
******************************************************************************/
#include "DEV_Config.h"
#include <fcntl.h>              // open(), O_RDWR
#include <sys/ioctl.h>          // ioctl()
#include <linux/spi/spidev.h>   // struct spi_ioc_transfer, SPI_MODE_1
#include <string.h>             // memset()
#include <stdio.h>
#include <unistd.h>
#include <bcm2835.h>            // bcm2835 라이브러리 필수

// --------------------------------------------------------------------------
// [중요] 핀 맵핑 설정
// CAN HAT이 SPI0의 CE0(Pin 24), CE1(Pin 26)을 모두 점유할 수 있으므로
// ADS1263의 CS 핀을 GPIO 22(Pin 15) 같은 비충돌 핀으로 연결했다고 가정합니다.
// 실제 하드웨어 연결에 맞춰 이 번호를 수정하세요.
// --------------------------------------------------------------------------
int DEV_RST_PIN  = 18;  // BCM GPIO 18 (Pin 12)
int DEV_CS_PIN   = 22;  // BCM GPIO 22 (Pin 15) - CAN 충돌 회피용
int DEV_DRDY_PIN = 17;  // BCM GPIO 17 (Pin 11)

// SPI 장치 파일 경로 (CAN이 0.0, ADC는 0.1 사용)
// 0.1은 물리적으로 CE1 핀과 연동되지만, SPI_NO_CS를 쓰므로 데이터 라인(MISO/MOSI/CLK)만 공유합니다.
#define SPI_DEVICE "/dev/spidev0.1"

// SPI 파일 디스크립터
int spi_fd = -1;

// --------------------------------------------------------------------------
// 유틸리티 함수 구현
// --------------------------------------------------------------------------

void DEV_Digital_Write(UWORD Pin, UBYTE Value)
{
    bcm2835_gpio_write(Pin, Value);
}

UBYTE DEV_Digital_Read(UWORD Pin)
{
    return bcm2835_gpio_lev(Pin);
}

// bcm2835_delay는 밀리초 단위 대기
void DEV_Delay_ms(UDOUBLE xms)
{
    bcm2835_delay(xms);
}

// 마이크로초 단위 대기 (필요시 사용)
void DEV_Delay_us(UDOUBLE xus)
{
    bcm2835_delayMicroseconds(xus);
}

/**
 * SPI 데이터 전송 (bcm2835 대신 Kernel spidev 사용)
 * ADS1263은 CS를 Low로 내린 상태에서 데이터를 주고받아야 합니다.
 * 상위 라이브러리(ADS1263.c)에서 CS 제어를 하겠지만,
 * 만약 안 한다면 여기서 CS 제어를 추가해야 합니다.
**/
UBYTE DEV_SPI_WriteByte(uint8_t Value)
{
    int ret;
    uint8_t tx[] = {Value};
    uint8_t rx[] = {0, };
    struct spi_ioc_transfer tr;

    // 구조체 초기화 (필수)
    memset(&tr, 0, sizeof(tr));

    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = 1;
    tr.delay_usecs = 0;
    tr.speed_hz = 2000000; // 2MHz (ADS1263 안정성 확보)
    tr.bits_per_word = 8;
    // tr.cs_change = 0; 

    if (spi_fd < 0) return 0;

    // 커널에게 SPI 전송 요청
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        perror("Failed to send SPI message");
        return 0;
    }

    return rx[0];
}

UBYTE DEV_SPI_ReadByte(void)
{
    return DEV_SPI_WriteByte(0x00); // Dummy byte 전송 후 수신
}

// --------------------------------------------------------------------------
// 초기화 및 GPIO 설정 함수 (누락되었던 부분 구현)
// --------------------------------------------------------------------------
void DEV_GPIO_Init(void)
{
    // CS 핀: 출력, 초기값 High (비활성)
    bcm2835_gpio_fsel(DEV_CS_PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(DEV_CS_PIN, HIGH);

    // RST 핀: 출력, 초기값 High
    bcm2835_gpio_fsel(DEV_RST_PIN, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_write(DEV_RST_PIN, HIGH);

    // DRDY 핀: 입력, Pull-up (데이터 준비 완료 시 Low로 떨어짐)
    bcm2835_gpio_fsel(DEV_DRDY_PIN, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_set_pud(DEV_DRDY_PIN, BCM2835_GPIO_PUD_UP);
}

/******************************************************************************
function:   Module Initialize
******************************************************************************/
UBYTE DEV_Module_Init(void)
{
    // 1. bcm2835 라이브러리 초기화 (GPIO 제어용)
    if(!bcm2835_init()) {
        printf("bcm2835 init failed! Try running with sudo.\r\n");
        return 1;
    }

    // 2. GPIO 핀 모드 설정 (위에서 구현한 함수 호출)
    DEV_GPIO_Init();

    // 3. spidev 초기화 (SPI 통신용)
    spi_fd = open(SPI_DEVICE, O_RDWR);
    if (spi_fd < 0) {
        printf("Failed to open SPI device: %s\r\n", SPI_DEVICE);
        printf("Check if 'dtparam=spi=on' is in /boot/config.txt\r\n");
        // SPI 실패해도 GPIO 제어는 닫고 리턴
        bcm2835_close();
        return 1;
    }

    // SPI 모드 설정 (Mode 1 + NO_CS)
    // SPI_NO_CS: 커널이 자동으로 CS 핀을 건드리는 것을 막습니다.
    // 대신 우리는 DEV_CS_PIN (GPIO 22)을 수동으로 제어합니다.
    uint8_t mode = SPI_MODE_1 | SPI_NO_CS;
    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("Failed to set SPI mode");
        close(spi_fd);
        bcm2835_close();
        return 1;
    }
    
    // SPI 속도 설정
    uint32_t speed = 2000000; // 2MHz
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("Failed to set SPI speed");
        close(spi_fd);
        bcm2835_close();
        return 1;
    }

    DEV_Delay_ms(100);
    printf("Hybrid Driver Init Success (SPI: %s, CS: GPIO%d)\r\n", SPI_DEVICE, DEV_CS_PIN);
    return 0;
}

void DEV_Module_Exit(void)
{
    // 종료 시 핀 상태 정리
    DEV_Digital_Write(DEV_RST_PIN, 0);
    DEV_Digital_Write(DEV_CS_PIN, 1); // CS High로 올려서 선택 해제

    if(spi_fd >= 0) {
        close(spi_fd);
        spi_fd = -1;
    }
    bcm2835_close();
}