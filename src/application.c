/* Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/location.h>
#include <nrf_modem_at.h>
#include <nrf_errno.h>
#include <net/nrf_cloud.h>
#include <net/nrf_cloud_alerts.h>
#include <date_time.h>
#include <cJSON.h>
#include <stdio.h>
#include <nrf_error.h>


#include "nrf_cloud_codec.h"
#include "application.h"
#include "temperature.h"
#include "connection.h"

#include "location_tracking.h"
#include "led_control.h"
#include <zephyr/net/socket.h>



#include <dk_buttons_and_leds.h>
#include <zephyr/sys/byteorder.h>

#include <net/nrf_cloud.h>
#include <zephyr/logging/log.h>

#include <zephyr/types.h>
#include <string.h>
#include <zephyr/irq.h>

#include "aggregator.h"
#include "alarm.h"

#include <zephyr/drivers/uart.h>


#include <sys/time.h>
#include <time.h>

#include <zephyr/drivers/adc.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

//#include <zephyr/fs/fs.h>
//#include <zephyr/fs/littlefs.h>
//#include <zephyr/storage/flash_map.h>

/* Matches LFS_NAME_MAX */

struct cameraControl cameraCtr;
struct boardStatus bStatus;

/*
uint32_t crc32_compute(uint8_t const * p_data, uint32_t size, uint32_t const * p_crc)
{
    uint32_t crc;

    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (uint32_t i = 0; i < size; i++)
    {
        crc = crc ^ p_data[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}
*/

#define UDP_IP_HEADER_SIZE 28

#define MSG_SIZE 1

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define GPIO_NODE        DT_NODELABEL(gpio0)


/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10000, 1);

//static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));


static int client_fd;
static int client_fd_stat;

static struct sockaddr_storage host_addr;
static struct sockaddr_storage host_addr_stat;

static struct k_work_delayable server_transmission_work;
static struct k_work_delayable rpi_power_off_work;

static struct k_work_delayable uart_work;

int16_t bat_voltage = 0;

LOG_MODULE_REGISTER(application, CONFIG_MQTT_MULTI_SERVICE_LOG_LEVEL);

/* Timer used to time the sensor sampling rate. */
static K_TIMER_DEFINE(sensor_sample_timer, NULL, NULL);

/* AT command request error handling */
#define AT_CMD_REQUEST_ERR_FORMAT "Error while processing AT command request: %d"
#define AT_CMD_REQUEST_ERR_MAX_LEN (sizeof(AT_CMD_REQUEST_ERR_FORMAT) + 20)
BUILD_ASSERT(CONFIG_AT_CMD_REQUEST_RESPONSE_BUFFER_LENGTH >= AT_CMD_REQUEST_ERR_MAX_LEN,
	     "Not enough AT command response buffer for printing error events.");

/* Temperature alert limits. */
#define TEMP_ALERT_LIMIT CONFIG_TEMP_ALERT_LIMIT
#define TEMP_ALERT_HYSTERESIS 2
#define TEMP_ALERT_LOWER_LIMIT (TEMP_ALERT_LIMIT + TEMP_ALERT_HYSTERESIS)

#define BATVOLT_R1 4.7f                 // MOhm
#define BATVOLT_R2 10.0f                // MOhm
#define INPUT_VOLT_RANGE 3.6f           // Volts
#define VALUE_RANGE_10_BIT 1.023        // (2^10 - 1) / 1000

#define ADC_NODE DT_NODELABEL(adc)

#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1_6
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
#define ADC_1ST_CHANNEL_ID 0
#define ADC_1ST_CHANNEL_INPUT SAADC_CH_PSELP_PSELP_AnalogInput0

#define BUFFER_SIZE 1
static int16_t m_sample_buffer[BUFFER_SIZE];
/*
#define RPI_NODE	DT_ALIAS(rpienable) // 
#if DT_NODE_HAS_STATUS(RPI_NODE, okay)
#define RPI_GPIO_LABEL	DT_GPIO_LABEL(RPI_NODE, gpios)
#define RPI_GPIO_PIN	DT_GPIO_PIN(RPI_NODE, gpios)
#define RPI_GPIO_FLAGS	(GPIO_OUTPUT | DT_GPIO_FLAGS(RPI_NODE, gpios))
#else
#error "Unsupported board: INA devicetree alias is not defined"
#define RPI_GPIO_LABEL	""
#define RPI_GPIO_PIN	0
#define RPI_GPIO_FLAGS	0
#endif
*/

#define GPIO0_LABEL DT_PROP(DT_NODELABEL(gpio0), label)
#define GPIO0_STATUS DT_PROP(DT_NODELABEL(gpio0), status)
#define PIN 16
#define FLAGS 0


static const struct device *adc_dev;

static const struct adc_channel_cfg m_1st_channel_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQUISITION_TIME,
	.channel_id = ADC_1ST_CHANNEL_ID,
	.input_positive   = ADC_1ST_CHANNEL_INPUT,
};

static int get_battery_voltage(uint16_t *battery_voltage)
{
	int err;

	const struct adc_sequence sequence = {
		.channels = BIT(ADC_1ST_CHANNEL_ID),
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer), // in bytes!
		.resolution = ADC_RESOLUTION,
	};

	if (!adc_dev) {
		return -1;
	}

	err = adc_read(adc_dev, &sequence);
	if (err) {
		LOG_INF("ADC read err: %d\n", err);

		return err;
	}

	float sample_value = 0;
	for (int i = 0; i < BUFFER_SIZE; i++) {
		sample_value += (float) m_sample_buffer[i];
	}
	sample_value /= BUFFER_SIZE;

	*battery_voltage = (uint16_t)(sample_value * (INPUT_VOLT_RANGE / VALUE_RANGE_10_BIT) * ((BATVOLT_R1 + BATVOLT_R2) / BATVOLT_R2));

	return 0;
}

bool init_adc()
{
	int err;

	adc_dev = DEVICE_DT_GET(ADC_NODE);
	if (!adc_dev) {
		LOG_INF("Error getting adc failed\n");

		return false;
	}

	err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
	if (err) {
		LOG_INF("Error in adc setup: %d\n", err);

		return false;
	}

	return true;
}
/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

typedef enum
{
        PAYLOAD_FILE_OPCODE_PING = 0xA0,
        PAYLOAD_FILE_OPCODE_FILEINFO_RSP = 0xA1,
        PAYLOAD_FILE_OPCODE_MTU_REQ = 0xA2,
        PAYLOAD_FILE_OPCODE_MTU_RSP = 0xA3,
        PAYLOAD_FILE_OPCODE_DATA_REQ = 0xA4,
        PAYLOAD_FILE_OPCODE_DATA_RSP = 0xA5,
        PAYLOAD_FILE_OPCODE_COMPLETE = 0xA6,
        PAYLOAD_FILE_OPCODE_COMPLETE_ACK = 0xA7,
        PAYLOAD_FILE_OPCODE_DATA_RSP_LAST = 0xA8,
} serial_payload_file_cmd_t;

typedef enum
{
        eSERIAL_FILE_OBJECT_STATE_IDLE = 0x00,
        eSERIAL_FILE_OBJECT_STATE_PING,
        eSERIAL_FILE_OBJECT_STATE_GET_FILE_INFO,
        eSERIAL_FILE_OBJECT_STATE_MTU_REQ,
        eSERIAL_FILE_OBJECT_STATE_MTU_RSP,
        eSERIAL_FILE_OBJECT_STATE_DATA_REQ,
        eSERIAL_FILE_OBJECT_STATE_DATA_RSP,
        eSERIAL_FILE_OBJECT_STATE_BLE_TX,
        eSERIAL_FILE_OBJECT_STATE_BLE_COMPLETE,
        eSERIAL_FILE_OBJECT_STATE_DONE
} serial_file_object_state_t;

typedef struct udp_packet_s
{
        uint16_t message_len;
		uint32_t sequenceNumber;
		bool last;
		uint32_t filesize;
    //    uint32_t crc32;
	//	uint32_t hash;
		uint8_t md5[16];
        char message[CONFIG_CAMERA_MTU];
        // serial_payload_file_cmd_t state;
} udp_packet_t;


typedef struct serial_file_object_s
{
        uint32_t filesize;
        uint32_t crc32;
        uint32_t offset_req;
        uint32_t offset_rsp;
        uint32_t length_req;
        uint8_t mtusize;
		uint8_t md5[16];
		uint32_t batteryVoltage;
		uint32_t batteryPercent;

        // serial_payload_file_cmd_t state;
} serial_file_object_t;

typedef struct serial_object_data_request_s
{
        uint8_t payload_len;
        serial_file_object_state_t opcode;
        uint16_t request_len;
        uint32_t offset_addr;
} serial_object_data_request_t;


enum {
        APP_CMD_NOCOMMAND = 0,
        APP_CMD_SINGLE_CAPTURE,
        APP_CMD_START_STREAM,
        APP_CMD_STOP_STREAM,
        APP_CMD_CHANGE_RESOLUTION,
        APP_CMD_CHANGE_PHY,
        APP_CMD_SEND_BLE_PARAMS,
        APP_CMD_SEND_BUFFER_REQ,
        APP_CMD_SEND_BUFFER_COMPLETE,
};

static serial_file_object_t m_file_object = { 0 };

static serial_object_data_request_t m_data_request_obj;
bool startSending = false;
bool lastFrame = false;


//const struct device *const dev = DEVICE_DT_GET_ONE(maxim_max17262);


#define GPIO_PORT DEVICE_DT_GET(DT_NODELABEL(gpio0))


typedef struct gpio_s {
    struct device const *port;
    gpio_pin_t const pin;
    gpio_flags_t flags;
} gpio_t;

typedef struct gpio_control_s {
    gpio_t solarChargerEnable;
    gpio_t solarChargerStatus;
	gpio_t solarChargerPGood;
    gpio_t rpiChargerEnable;
    gpio_t rpiChargerLow;
	gpio_t icarusCharger;
	gpio_t rpiShutdown;
    gpio_t e;
    gpio_t f;
    gpio_t g;
} gpio_control_t;

static gpio_control_t gpio_control = {
  .solarChargerEnable = {
    .port = GPIO_PORT,
    .pin = 0,
    .flags = GPIO_OUTPUT_LOW
  },
  .solarChargerStatus = {
    .port = GPIO_PORT,
    .pin = 1,
    .flags = GPIO_INPUT
  },
    .solarChargerPGood = {
    .port = GPIO_PORT,
    .pin = 2,
    .flags = GPIO_INPUT
  },
	.rpiChargerEnable = {
    .port = GPIO_PORT,
    .pin = 3,
    .flags = GPIO_OUTPUT_LOW
  },
	.rpiChargerLow = {
    .port = GPIO_PORT,
    .pin = 4,
    .flags = GPIO_INPUT
  },
	.icarusCharger = {
    .port = GPIO_PORT,
    .pin = 7,
    .flags = GPIO_OUTPUT_LOW
  },
  	.rpiShutdown = {
    .port = GPIO_PORT,
    .pin = 30,
    .flags = GPIO_OUTPUT_HIGH
  },
};

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	while (uart_irq_rx_ready(uart_dev)) {

		uart_fifo_read(uart_dev, &c, 1);
		k_msgq_put(&uart_msgq, &c, K_NO_WAIT);
	}
	k_work_schedule(&uart_work, K_NO_WAIT);

}

/*
 * Print a null-terminated string character by character to the UART interface
 */

#define PRINT_HEX(p_label, p_text, len)\
	({\
		LOG_INF("---- %s (len: %u): ----", p_label, len);\
		LOG_HEXDUMP_INF(p_text, len, "Content:");\
		LOG_INF("---- %s end  ----", p_label);\
	})



void print_uart(char *buf, int msg_len)
{
//	int msg_len = strlen(buf);
//	LOG_INF("%d",msg_len);

	for (int i = 0; i < msg_len; i++) {
	//	LOG_INF("%c",buf[i]);
		uart_poll_out(uart_dev, buf[i]);
	}
}

void print_uart_struct(char *buf, int msg_len)
{
//	int msg_len = strlen(buf);
	//LOG_INF("%d",msg_len);
//	PRINT_HEX("UART message:",buf, sizeof(buf));

	for (int i = 0; i < (msg_len*2); i++) {
//		LOG_INF("%d",buf[i]);
		//uart_poll_out(uart_dev, buf[i]);
		uart_poll_out(uart_dev, buf[i]);
	}
}

/* Buffer to contain modem responses when performing AT command requests */
static char at_req_resp_buf[CONFIG_AT_CMD_REQUEST_RESPONSE_BUFFER_LENGTH];

/**
 * @brief Construct a device message cJSON object with automatically generated timestamp
 *
 * The resultant cJSON object will be conformal to the General Message Schema described in the
 * application-protocols repo:
 *
 * https://github.com/nRFCloud/application-protocols
 *
 * @param appid - The appId for the device message
 * @param msg_type - The messageType for the device message
 * @return cJSON* - the timestamped data device message object if successful, NULL otherwise.
 */

#include <psa/crypto.h>
#include <psa/crypto_extra.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_BUILD_WITH_TFM
#include <tfm_ns_interface.h>
#endif

#define APP_SUCCESS		0
#define APP_ERROR		-1
#define APP_SUCCESS_MESSAGE "Example finished successfully!"
#define APP_ERROR_MESSAGE "Example exited with error!"

/* ====================================================================== */
/*				Global variables/defines for the SHA256 example			  */

#define NRF_CRYPTO_EXAMPLE_SHA256_TEXT_SIZE 150
#define NRF_CRYPTO_EXAMPLE_SHA256_SIZE 32

char tx_buf[MSG_SIZE];


/* Below text is used as plaintext for computing/verifying the hash. 
static uint8_t m_plain_text[NRF_CRYPTO_EXAMPLE_SHA256_TEXT_SIZE] = {
	"Example string to demonstrate basic usage of SHA256."
	"That uses single and multi-part PSA crypto API's to "
	"perform a SHA-256 hashing operation."
};

static uint8_t m_hash[NRF_CRYPTO_EXAMPLE_SHA256_SIZE];
*/

int handshake = 0;
bool firstByte = true;
char uartMessage[10000];
int receiveIndex = 0;
int tempIndex = 0;
int frameIndex = 0;

int messageLen=0;
struct udp_packet_s udpPacket = {};
serial_payload_file_cmd_t messageType;



/* ====================================================================== */

static int crypto_init(void)
{
	psa_status_t status;

	/* Initialize PSA Crypto */
	
	status = psa_crypto_init();
	if (status != PSA_SUCCESS)
		return -1;

	return 0;
}
/*
int hash_singlepart_sha256()
{
	uint32_t olen;
	psa_status_t status;

	LOG_INF("Hashing using SHA256...");
*/
	/* Calculate the SHA256 hash */
	/*
	memset(m_hash, 0, sizeof(m_hash));
	memset(newMess.m_hash, 0, sizeof(m_hash));

	status = psa_hash_compute(
		PSA_ALG_SHA_256, (struct TextMessage*)&newMess, sizeof((struct TextMessage*)&newMess), m_hash, sizeof(m_hash), &olen);
	if (status != PSA_SUCCESS) {
		LOG_INF("psa_hash_compute failed! (Error: %d)", status);
		return -1;
	}

	LOG_INF("Hashing successful!");
	PRINT_HEX("SHA256 hash", m_hash, sizeof(m_hash));

	return 0;
}
*/
/*
int hash_multipart_sha256(void)
{
	uint32_t olen;
	psa_status_t status;
	uint8_t *input_ptr = (struct TextMessage*)&newMess;
	psa_hash_operation_t hash_operation = {0};
	memset(newMess.m_hash, 0, sizeof(m_hash));

	LOG_INF("Hashing using multi-part SHA256...");
*/
	/* Setup a multipart hash operation */
	/*
	status = psa_hash_setup(&hash_operation, PSA_ALG_SHA_256);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Could not setup the hash operation! Error %d", status);
		return -1;
	}
*/

	/* Feed the chunks of the input data to the PSA driver */
/*	
	status = psa_hash_update(&hash_operation, input_ptr, 42);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Could not hash the next chunk! Error %d", status);
		return -1;

	}
	LOG_INF("Added %d bytes", 42);
	input_ptr += 42;


	status = psa_hash_update(&hash_operation, input_ptr, 58);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Could not hash the next chunk! Error %d", status);
		return -1;

	}
	LOG_INF("Added %d bytes", 58);
	input_ptr += 58;

	status = psa_hash_update(&hash_operation, input_ptr, 50);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Could not hash the next chunk! Error %d", status);
		return -1;

	}
	LOG_INF("Added %d bytes", 50);

	status = psa_hash_finish(&hash_operation, m_hash, sizeof(m_hash), &olen);
	if (status != PSA_SUCCESS) {
		LOG_ERR("Could not finish the hash operation! Error %d", status);
		return -1;
	}

	LOG_INF("Hashing successful!");
	PRINT_HEX("SHA256 hash", m_hash, sizeof(m_hash));

	return 0;
}
*/
/*
int verify_sha256(void)
{
	psa_status_t status;

	LOG_DBG("Verifying the SHA256 hash...");

	/* Verify the hash */
	/*
	status = psa_hash_compare(
		PSA_ALG_SHA_256, m_plain_text, sizeof(m_plain_text), m_hash, sizeof(m_hash));
	if (status != PSA_SUCCESS) {
		LOG_DBG("psa_hash_compare failed! (Error: %d)", status);
		return APP_ERROR;
	}

	LOG_DBG("SHA256 verification successful!");

	return APP_SUCCESS;
}
*/



static int server_init(void)
{
	struct sockaddr_in *server4 = ((struct sockaddr_in *)&host_addr);

	server4->sin_family = AF_INET;
	server4->sin_port = htons(CONFIG_UDP_SERVER_PORT);

	inet_pton(AF_INET, CONFIG_UDP_SERVER_ADDRESS_STATIC,
		  &server4->sin_addr);

	return 0;
}

static int server_init_stat(void)
{
	struct sockaddr_in *server4 = ((struct sockaddr_in *)&host_addr_stat);

	server4->sin_family = AF_INET;
	server4->sin_port = htons(CONFIG_UDP_SERVER_PORT_STAT);

	inet_pton(AF_INET, CONFIG_UDP_SERVER_ADDRESS_STATIC,
		  &server4->sin_addr);

	return 0;
}

static void server_disconnect(void)
{
	(void)close(client_fd);
}

static void server_disconnect_stat(void)
{
	(void)close(client_fd_stat);
}

static int server_connect_stat(void)
{
	int err;

	client_fd_stat = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (client_fd_stat < 0) {
		LOG_ERR("Failed to create UDP socket: %d\n", errno);
		err = -errno;
		goto error;
	}

	err = connect(client_fd_stat, (struct sockaddr *)&host_addr_stat,
		      sizeof(struct sockaddr_in));
	if (err < 0) {
		LOG_ERR("Connect failed : %d\n", errno);
		goto error;
	}

	return 0;

error:
	server_disconnect_stat();

	return err;
}


static int server_connect(void)
{
	int err;

	client_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (client_fd < 0) {
		LOG_ERR("Failed to create UDP socket: %d\n", errno);
		err = -errno;
		goto error;
	}

	err = connect(client_fd, (struct sockaddr *)&host_addr,
		      sizeof(struct sockaddr_in));
	if (err < 0) {
		LOG_ERR("Connect failed : %d\n", errno);
		goto error;
	}

	return 0;

error:
	server_disconnect();

	return err;
}

static int powerSave()
{
//	if(bStatus.)
	int status = 255;
	struct timeval tv;
    struct timezone tz;
    struct tm *today;
    int zone;//	LOG_INF("CRC: %d",crc);
    gettimeofday(&tv, &tz);
    today = localtime(&tv.tv_sec);
	if((today->tm_hour >CONFIG_CAMERA_START_HOUR)&&(today->tm_hour)<CONFIG_CAMERA_FINISH_HOUR)
	{
		if(!bStatus.solarChargerPGood)
		{
			gpio_pin_set(GPIO_PORT,gpio_control.rpiShutdown.pin,0);
			bStatus.rpiShutdown = 1;
		//	cameraCtr.powerSave = true;
			LOG_INF("shutting down RPI leaving power to nrf on");
			status = 1;
			int icarusCharger = gpio_pin_set(GPIO_PORT,gpio_control.icarusCharger.pin,0);
			bStatus.icarusChargerEnable = true;
			LOG_INF("Icarus charger status  PS: %d / %d",icarusCharger,bStatus.icarusChargerEnable);
			return status;
		}
	}
	
	gpio_pin_set(GPIO_PORT,gpio_control.rpiShutdown.pin,0);
	k_work_schedule(&rpi_power_off_work,K_SECONDS(60));
	//	cameraCtr.powerSave = false;
	bStatus.rpiShutdown = 1;
	int icarusCharger = gpio_pin_set(GPIO_PORT,gpio_control.icarusCharger.pin,1);
	bStatus.icarusChargerEnable = false;

	LOG_INF("Icarus charger status NON PS: %d / %d",icarusCharger,bStatus.icarusChargerEnable);
	LOG_INF("Shutting down RPI and power");

	

	return status;
}

static void send_udp()
{

	int err;
	startSending = false;
//	int crc = crc32_compute((uint8_t*)uartMessage, receiveIndex, NULL);
	if(receiveIndex==m_file_object.filesize)
	{
		udpPacket.last=true;
		udpPacket.message_len = m_data_request_obj.request_len;

	}
	else
	{
		udpPacket.last=false;
		udpPacket.message_len = m_data_request_obj.request_len;


	}

//	udpPacket.crc32= crc;
	udpPacket.filesize=m_file_object.filesize;
//	udpPacket.message_len=512;
	memcpy(udpPacket.message, &uartMessage[0],CONFIG_CAMERA_MTU);

	err = send(client_fd, (struct udp_packet_s*)&udpPacket, sizeof(udpPacket), 0);

	udpPacket.sequenceNumber++;

	if (err < 0) {
	LOG_ERR("Failed to transmit UDP packet, %d\n", errno);
	}
	m_data_request_obj.payload_len = 8;
	m_data_request_obj.opcode = PAYLOAD_FILE_OPCODE_DATA_REQ;
    m_data_request_obj.offset_addr = receiveIndex;

	if(m_file_object.filesize > m_data_request_obj.offset_addr+CONFIG_CAMERA_MTU)
	{
		m_data_request_obj.request_len=1024;	
	}
	else
	{
		m_data_request_obj.request_len =
                m_file_object.filesize - m_data_request_obj.offset_addr ;
	}
	if(receiveIndex!=m_file_object.filesize)
	{
        print_uart_struct((struct serial_object_data_request_t*)&m_data_request_obj, sizeof((struct serial_object_data_request_t*)&m_data_request_obj));
 //       LOG_INF("PAYLOAD_FILE_OPCODE_DATA_REQ offset_addr= %x, %x",
 //                             m_data_request_obj.offset_addr, m_data_request_obj.request_len);
	}
	else
	{
		cameraCtr.takePictureNow = false;
		cameraCtr.processed = true;
		cameraCtr.processing = false;
		handshake = 0;
		receiveIndex = 0;
		tempIndex = 0;
		frameIndex = 0;
		firstByte = true;
		udpPacket.sequenceNumber=0;
		date_time_now(&cameraCtr.lastPictureTaken);
		int status = powerSave();

	//	k_work_schedule(&uart_work, K_SECONDS(1200));

	}


}



static void server_transmission_work_fn(struct k_work *work)
{
	int err;
	int err2;

	int status;
	struct timeval tv;
    struct timezone tz;
    struct tm *today;
    int zone;//	LOG_INF("CRC: %d",crc);
    gettimeofday(&tv, &tz);
    today = localtime(&tv.tv_sec);
	LOG_INF("hour now %d",today->tm_hour);

//	char buffer[CONFIG_UDP_DATA_UPLOAD_SIZE_BYTES] = {"10,10,10"};
	//if
  	//gpio_pin_set(three_digit.d1_en.port,three_digit.d1_en.pin,0);

	LOG_INF("Transmitting UDP/IP payload of %d bytes to the ",
	       CONFIG_UDP_DATA_UPLOAD_SIZE_BYTES + UDP_IP_HEADER_SIZE);
	LOG_INF("IP address %s, port number %d\n",
	       CONFIG_UDP_SERVER_ADDRESS_STATIC,
	       CONFIG_UDP_SERVER_PORT);
	LOG_INF("Receive Index %d ",receiveIndex);
	if(receiveIndex!=0 || handshake ==1 || cameraCtr.processing )
	{
		handshake = 0;
		receiveIndex = 0;
		tempIndex = 0;
		frameIndex = 0;
		firstByte = true;

	}
//	cameraCtr.takePictureNow = true;

  	int solarChargerStatus = gpio_pin_get(GPIO_PORT,gpio_control.solarChargerStatus.pin);
  	int solarChargerPGood = gpio_pin_get(GPIO_PORT,gpio_control.solarChargerPGood.pin);
 // 	int solarChargerEnable = gpio_pin_get(GPIO_PORT,gpio_control.solarChargerEnable.pin);
//	int solarChargerEnable = bStatus.solarChargerEnable;
  	int rpiChargerLow = gpio_pin_get(GPIO_PORT,gpio_control.rpiChargerLow.pin);
//	int rpiChargerEnable = gpio_pin_get(GPIO_PORT,gpio_control.rpiChargerEnable.pin);

	LOG_INF("Solar charger status: %d",solarChargerStatus);
	LOG_INF("Solar charger status Pgood: %d",solarChargerPGood);
//	LOG_INF("Solar charger status Enable: %d",solarChargerEnable);

	LOG_INF("rpi low status: %d",rpiChargerLow);
	LOG_INF("rpi charger enable status: %d",bStatus.rpiChargerEnable);

	bStatus.solarChargerStatus = solarChargerStatus;
	bStatus.solarChargerPGood = solarChargerPGood;
//	bStatus.solarChargerEnable = solarChargerEnable;
	bStatus.rpiChargerLow = rpiChargerLow;
	//bStatus.rpiChargerEnable = rpiChargerEnable;
	bStatus.lastPictureTaken = cameraCtr.lastPictureTaken;
	bStatus.nextScheduledTime = cameraCtr.nextScheduledTime;
	bStatus.solarChargerEnable = cameraCtr.solarChargerEnable;

	//int change = gpio_pin_toggle(GPIO_PORT,gpio_control.solarChargerEnable.pin);
	if((bStatus.solarChargerEnable==255))
	{
		gpio_pin_set(GPIO_PORT,gpio_control.solarChargerEnable.pin,1);
//		gpio_pin_set(GPIO_PORT,gpio_control.icarusCharger.pin,0);
//		bStatus.icarusChargerEnable = true;
	}
	else
	{
		gpio_pin_set(GPIO_PORT,gpio_control.solarChargerEnable.pin,0);
	}
	if(!bStatus.solarChargerPGood &&(bStatus.rpiShutdown==1))
	{
		int icarusCharger = gpio_pin_set(GPIO_PORT,gpio_control.icarusCharger.pin,0);
		bStatus.icarusChargerEnable = true;
		LOG_INF("Icarus charger status work1: %d / %d",icarusCharger,bStatus.icarusChargerEnable);
	}
	else
	{
		int icarusCharger = gpio_pin_set(GPIO_PORT,gpio_control.icarusCharger.pin,1);
		bStatus.icarusChargerEnable = false;
		LOG_INF("Icarus charger status work2: %d / %d",icarusCharger,bStatus.icarusChargerEnable);


	}

	if(cameraCtr.takePictureNow)
	{
		k_work_schedule(&uart_work, K_NO_WAIT);
		if(bStatus.rpiShutdown==1)
		{
			gpio_pin_set(GPIO_PORT,gpio_control.rpiChargerEnable.pin,0);
			bStatus.rpiChargerEnable = 0;
	//		gpio_pin_set(GPIO_PORT,gpio_control.rpiChargerEnable.pin,1);
	//		cameraCtr.powerSave = false;
			bStatus.rpiShutdown = 2;

			LOG_INF("Walking up rpi from powersave");

		}
		else
		{
			gpio_pin_set(GPIO_PORT,gpio_control.rpiChargerEnable.pin,1);
			bStatus.rpiChargerEnable = 1;

			bStatus.rpiShutdown = 0;
			LOG_INF("Enabling RPI");

		}

	}
	else
	{
		u_int64_t timestamp;
		date_time_now(&timestamp);
		if((today->tm_hour >CONFIG_CAMERA_START_HOUR)&&(today->tm_hour)<CONFIG_CAMERA_FINISH_HOUR)
		{
			
			if(timestamp > (cameraCtr.lastPictureTaken + (cameraCtr.cameraFrequency*60*1000)))
			{
				cameraCtr.takePictureNow = true;
				k_work_schedule(&uart_work, K_NO_WAIT);
				if(bStatus.rpiShutdown==1)
				{
					gpio_pin_set(GPIO_PORT,gpio_control.rpiChargerEnable.pin,0);
					bStatus.rpiChargerEnable = 0;

				//	gpio_pin_set(GPIO_PORT,gpio_control.rpiChargerEnable.pin,1);
			//		cameraCtr.powerSave = false;
					bStatus.rpiShutdown = 2;
					LOG_INF("Walking up rpi from powersave");
				}
				else
				{
					gpio_pin_set(GPIO_PORT,gpio_control.rpiChargerEnable.pin,1);
					bStatus.rpiChargerEnable = 1;

					LOG_INF("Enabling RPI");
					bStatus.rpiShutdown = 0;
					
				}			
			}
		}
		else
		{
			if(	bStatus.rpiChargerEnable == 1)
			{
				gpio_pin_set(GPIO_PORT,gpio_control.rpiShutdown.pin,0);
		//		cameraCtr.powerSave = false;
				bStatus.rpiShutdown  = 1;
				k_work_schedule(&rpi_power_off_work,K_SECONDS(60));
				LOG_INF("Gracefully shutting down pi and turining it off");
			}
			else
			{
				LOG_INF("RPI already off");
			}
		}

		
	}


/*		struct sensor_value voltage, avg_current, temperature;
		float i_avg;

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_GAUGE_VOLTAGE, &voltage);
		sensor_channel_get(dev, SENSOR_CHAN_GAUGE_AVG_CURRENT,
						  &avg_current);
		sensor_channel_get(dev, SENSOR_CHAN_GAUGE_TEMP, &temperature);

		i_avg = avg_current.val1 + (avg_current.val2 / 1000000.0);

		LOG_INF("V: %d.%06d V; I: %f mA; T: %d.%06d °C\n",
		      voltage.val1, voltage.val2, (double)i_avg,
		      temperature.val1, temperature.val2);
*/
	//int crc = crc32_compute((uint8_t*)uartMessage, receiveIndex, NULL);


	err2 = send(client_fd_stat, (struct boardStatus*)&bStatus, sizeof(bStatus), 0);
		if (err < 0) {
		LOG_ERR("Failed to transmit UDP packet, %d\n", errno);
	//	return;
		}
	k_work_schedule(&server_transmission_work,
			K_SECONDS(CONFIG_UDP_DATA_UPLOAD_FREQUENCY_SECONDS));

	//k_work_schedule(&uart_work, K_NO_WAIT);



}

static void rpi_power_off_work_fn(struct k_work *work)
{
		gpio_pin_set(GPIO_PORT,gpio_control.rpiChargerEnable.pin,0);
		bStatus.rpiChargerEnable = 0;

		LOG_INF("disabling RPI nad shutting down power to nrf");
	//	cameraCtr.powerSave = false;
		bStatus.rpiShutdown = 255;
}


static void uart_work_fn(struct k_work *work)
{
	int err;
	int status;
	uint8_t tx_buffer[16];
	uint16_t tx_buffer_len = 0;
    uint32_t err_code;
	bool startSending = false;
//	bool lastFrame = false;

 //   LOG_INF("Uart work queue started");
 if(cameraCtr.takePictureNow)
 {
	//LOG_INF("Talking picture now");

	if(handshake== 0)
	{
    	LOG_INF("Sending PING");

		tx_buffer_len = 5;
    	tx_buffer[0] = tx_buffer_len;
    	tx_buffer[1] = PAYLOAD_FILE_OPCODE_PING;
       //         tx_buffer[2] = m_ble_mtu_length;
    	tx_buffer[2] = 128;
		tx_buffer[3] = cameraCtr.delayUdpSend;
		tx_buffer[4] = cameraCtr.quality;
		print_uart(tx_buffer,5);
		handshake = 1;
		cameraCtr.processing = true;
		bStatus.delayUdpSend = cameraCtr.quality;
	}
	else
	{
//		LOG_INF("Entered handshake");
		int counter = 0;
		char len[1];
		char req[1];
		serial_payload_file_cmd_t messageType;


	//	bool firstByte = true;
	//	LOG_INF("receive index: %d",receiveIndex);
		while (k_msgq_get(&uart_msgq, &tx_buf, K_NO_WAIT) == 0)
		{
			if(firstByte && (handshake==1))
			{
				memcpy(&uartMessage[receiveIndex],&tx_buf,1);
				handshake=2;
				receiveIndex++;
				firstByte = false;
	//			LOG_INF("H2 first byte");
				continue;

			}
			int x = (int)(uartMessage[0]);
	
	//		LOG_INF("x: %d",x);

			if(receiveIndex<=x && (handshake==2))
			{
				memcpy(&uartMessage[receiveIndex],&tx_buf,1);
		//		LOG_INF("H2 copying/reading bytes");
				receiveIndex++;
				if(receiveIndex==(x+1))
				{
					handshake=3;
			//		LOG_INF("H2 done");
					receiveIndex=0;
					firstByte=true;
					m_file_object.filesize = ((int)uartMessage[2]);
                	m_file_object.filesize +=  ((int)uartMessage[3]) << 8;
                	m_file_object.filesize +=  ((int)uartMessage[4]) << 16;
                	m_file_object.filesize +=  ((int)uartMessage[5]) << 24;
                	m_file_object.offset_req = 0;
					LOG_INF("File Size = %d", m_file_object.filesize);
				//	char md5Array[32];
					for(int i = 0; i < 16; i++)
					{
						m_file_object.md5[i] =  (char)uartMessage[6+i];
					//	bStatus.md5Array[i] = (char)uartMessage[6+i];
						udpPacket.md5[i] = (char)uartMessage[6+i];
					//	LOG_INF("CRC32  = %d", m_file_object.md5[i]);

					}
				//		m_file_object.md5[i] =  (char)uartMessage[6+i];
					bStatus.lastPictureQuality = cameraCtr.quality;
					m_file_object.batteryVoltage = ((int)uartMessage[22]);
                	m_file_object.batteryVoltage +=  ((int)uartMessage[23]) << 8;
                	m_file_object.batteryVoltage +=  ((int)uartMessage[24]) << 16;
                	m_file_object.batteryVoltage +=  ((int)uartMessage[25]) << 24;

					bStatus.rpiBatteryVoltage = m_file_object.batteryVoltage;

					m_file_object.batteryPercent = ((int)uartMessage[26]);
                	m_file_object.batteryPercent +=  ((int)uartMessage[27]) << 8;
                	m_file_object.batteryPercent +=  ((int)uartMessage[28]) << 16;
                	m_file_object.batteryPercent +=  ((int)uartMessage[29]) << 24;

					bStatus.rpiBatteryPercent = m_file_object.batteryPercent;
             //   	LOG_INF("CRC32  = %s", m_file_object.md5);
                	LOG_INF("CRC32  = %d", m_file_object.batteryVoltage);
                	LOG_INF("CRC32  = %d", m_file_object.batteryPercent);

			//		for(int i = 0; i<=x; i++)
			//		{
			//			LOG_INF("%d",(int)uartMessage[i]);
			//		}

					m_data_request_obj.payload_len = 8;
                	m_data_request_obj.opcode = PAYLOAD_FILE_OPCODE_DATA_REQ;
                	m_data_request_obj.offset_addr = m_file_object.offset_req;
		//			udpPacket.crc32=m_file_object.crc32;
					if(m_file_object.filesize > m_data_request_obj.offset_addr+CONFIG_CAMERA_MTU)
					{
						m_data_request_obj.request_len=CONFIG_CAMERA_MTU;	
					}
					else
					{
						        m_data_request_obj.request_len =
                                m_file_object.filesize - m_file_object.offset_req;
					}
            //    	LOG_INF("PAYLOAD_FILE_OPCODE_DATA_REQ offset_addr= %x, %x",
             //                 m_data_request_obj.offset_addr, m_data_request_obj.request_len);

                	print_uart_struct((struct serial_object_data_request_t*)&m_data_request_obj, sizeof((struct serial_object_data_request_t*)&m_data_request_obj));
					continue;
					//(struct serial_object_data_request_t*)&m_data_request_obj, sizeof(m_data_request_obj)
				}
			}
			if(firstByte && (handshake==3) && !lastFrame)
			{
				memcpy(&messageLen,&tx_buf,1);
				handshake=4;
			//	receiveIndex++;
			//	tempIndex++;
				firstByte = false;
	//			LOG_INF("H3 first byte: %d", messageLen);
				continue;

			}
			if(handshake==4)
			{
				if(tempIndex ==0)
				{
					memcpy(&messageType,&tx_buf,1);

			//		LOG_INF("message type: %x", messageType);
					tempIndex++;
					if(messageType == PAYLOAD_FILE_OPCODE_DATA_RSP_LAST)
					{
						lastFrame = true;
			//			LOG_INF("Last frame %d", lastFrame);

					}

				}
				else
				{
					memcpy(&uartMessage[frameIndex],&tx_buf,1);
					receiveIndex++;
					tempIndex++;
					frameIndex++;
			//		LOG_INF("Last frame status in else %d", lastFrame);

					if(tempIndex == messageLen)
					{
						tempIndex=0;
						handshake=3;
						firstByte=true;
			//			LOG_INF("End of frame");
				//		LOG_INF("Last frame status in else %d", lastFrame);

						if(lastFrame)
						{
							startSending = true;
							lastFrame = false;
							frameIndex = 0;
					//		LOG_INF("start sending");
							send_udp();
						}
		
					}
				}
			}
		}
	}
 }
}


static cJSON *create_timestamped_device_message(const char *const appid, const char *const msg_type)
{
	cJSON *msg_obj = NULL;
	int64_t timestamp;

	/* Acquire timestamp */
	if (date_time_now(&timestamp)) {
		LOG_ERR("Failed to create timestamp for data message "
			"with appid %s", appid);
		return NULL;
	}

	/* Create container object */
	msg_obj = json_create_req_obj(appid, msg_type);
	if (msg_obj == NULL) {
		LOG_ERR("Failed to create container object for timestamped data message "
			"with appid %s and message type %s", appid, msg_type);
		return NULL;
	}

	/* Add timestamp to container object */
	if (!cJSON_AddNumberToObject(msg_obj, NRF_CLOUD_MSG_TIMESTAMP_KEY, (double)timestamp)) {
		LOG_ERR("Failed to add timestamp to data message with appid %s and message type %s",
			appid, msg_type);
		cJSON_Delete(msg_obj);
		return NULL;
	}

	return msg_obj;
}

/**
 * @brief Transmit a collected sensor sample to nRF Cloud.
 *
 * @param sensor - The name of the sensor which was sampled.
 * @param value - The sampled sensor value.
 * @return int - 0 on success, negative error code otherwise.
 */
static int send_sensor_sample(const char *const sensor, double value)
{
	int ret = 0;

	/* Create a timestamped message container object for the sensor sample. */
	cJSON *msg_obj = create_timestamped_device_message(
		sensor, NRF_CLOUD_JSON_MSG_TYPE_VAL_DATA
	);

	if (msg_obj == NULL) {
		ret = -EINVAL;
		goto cleanup;
	}

	/* Populate the container object with the sensor value. */
	if (cJSON_AddNumberToObject(msg_obj, NRF_CLOUD_JSON_DATA_KEY, value) == NULL) {
		ret = -ENOMEM;
		LOG_ERR("Failed to append value to %s sample container object ",
			sensor);
		goto cleanup;
	}

	/* Send the sensor sample container object as a device message. */
	ret = send_device_message_cJSON(msg_obj);

cleanup:
	if (msg_obj) {
		cJSON_Delete(msg_obj);
	}
	return ret;
}

/**
 * @brief Transmit a collected GNSS sample to nRF Cloud.
 *
 * @param loc_gnss - GNSS location data.
 * @return int - 0 on success, negative error code otherwise.
 */
static int send_gnss(const struct location_event_data * const loc_gnss)
{
	if (!loc_gnss || (loc_gnss->method != LOCATION_METHOD_GNSS)) {
		return -EINVAL;
	}

	int ret = 0;
	struct nrf_cloud_gnss_data gnss_pvt = {
		.type = NRF_CLOUD_GNSS_TYPE_PVT,
		.ts_ms = NRF_CLOUD_NO_TIMESTAMP,
		.pvt = {
			.lon		= loc_gnss->location.longitude,
			.lat		= loc_gnss->location.latitude,
			.accuracy	= loc_gnss->location.accuracy,
			.has_alt	= 0,
			.has_speed	= 0,
			.has_heading	= 0
		}
	};

	cJSON *msg_obj = cJSON_CreateObject();

	/* Add the timestamp */
	(void)date_time_now(&gnss_pvt.ts_ms);

	/* Encode the location data into a device message */
	ret = nrf_cloud_gnss_msg_json_encode(&gnss_pvt, msg_obj);

	if (ret == 0) {
		/* Send the location message */
		ret = send_device_message_cJSON(msg_obj);
	}

	if (msg_obj) {
		cJSON_Delete(msg_obj);
	}

	return ret;
}

/**
 * @brief Callback to receive periodic location updates from location_tracking.c and forward them
 * to nRF Cloud.
 *
 * Note that cellular positioning (MCELL/Multi-Cell and SCELL/Single-Cell) is sent to nRF
 * Cloud automatically (since the Location library and nRF Cloud must work together to
 * determine them in the first place). GNSS positions, on the other hand, must be
 * sent manually, since they are determined entirely on-device.
 *
 * @param location_data - The received location update.
 *
 */
static void on_location_update(const struct location_event_data * const location_data)
{
	LOG_INF("Location Updated: %.06f N %.06f W, accuracy: %.01f m, Method: %s",
		location_data->location.latitude,
		location_data->location.longitude,
		location_data->location.accuracy,
		location_method_str(location_data->method));

	/* If the position update was derived using GNSS, send it onward to nRF Cloud. */
	if (location_data->method == LOCATION_METHOD_GNSS) {
		LOG_INF("GNSS Position Update! Sending to nRF Cloud...");
		send_gnss(location_data);
	}
}

/**
 * @brief Receives general device messages from nRF Cloud, checks if they are AT command requests,
 * and performs them if so, transmitting the modem response back to nRF Cloud.
 *
 * Try sending {"appId":"MODEM", "messageType":"CMD", "data":"AT+CGMR"}
 * in the nRF Cloud Portal Terminal card.
 *
 * @param msg - The device message to check.
 */
static void handle_at_cmd_requests(const char *const msg)
{
	/* Attempt to parse the message as if it is JSON */
	struct cJSON *msg_obj = cJSON_Parse(msg);

	if (!msg_obj) {
		/* The message isn't JSON or otherwise couldn't be parsed. */
		LOG_DBG("A general topic device message of length %d could not be parsed.",
			msg ? strlen(msg) : 0);
		return;
	}

	/* Check that we are actually dealing with an AT command request */
	char *msg_appid =
		cJSON_GetStringValue(cJSON_GetObjectItem(msg_obj, NRF_CLOUD_JSON_APPID_KEY));
	char *msg_type =
		cJSON_GetStringValue(cJSON_GetObjectItem(msg_obj, NRF_CLOUD_JSON_MSG_TYPE_KEY));
	if (!msg_appid || !msg_type ||
	    (strcmp(msg_appid, NRF_CLOUD_JSON_APPID_VAL_MODEM)  != 0) ||
	    (strcmp(msg_type,  NRF_CLOUD_JSON_MSG_TYPE_VAL_CMD) != 0)) {
		goto cleanup;
	}

	/* If it is, extract the command string */
	char *cmd =
		cJSON_GetStringValue(cJSON_GetObjectItem(msg_obj, NRF_CLOUD_JSON_DATA_KEY));

	if (!cmd) {
		/* Missing or invalid command value will be treated as a blank command */
		cmd = "";
	}

	/* Execute the command and receive the result */
	LOG_DBG("Modem AT command requested: %s", cmd);
	memset(at_req_resp_buf, 0, sizeof(at_req_resp_buf));

	/* We must pass the command in using a format specifier it might contain special characters
	 * such as %.
	 *
	 * We subtract 1 from the passed-in response buffer length to ensure that the response is
	 * always null-terminated, even when the response is longer than the response buffer size.
	 */
	int err = nrf_modem_at_cmd(at_req_resp_buf, sizeof(at_req_resp_buf) - 1, "%s", cmd);

	LOG_DBG("Modem AT command response (%d, %d): %s",
		nrf_modem_at_err_type(err), nrf_modem_at_err(err), at_req_resp_buf);

	/* Trim \r\n from modem response for better readability in the portal. */
	at_req_resp_buf[MAX(0, strlen(at_req_resp_buf) - 2)] = '\0';

	/* If an error occurred with the request, report it */
	if (err < 0) {
		/* Negative error codes indicate an error with the modem lib itself, so the
		 * response buffer will be empty (or filled with junk). Thus, we can print the
		 * error message directly into it.
		 */
		snprintf(at_req_resp_buf, sizeof(at_req_resp_buf), AT_CMD_REQUEST_ERR_FORMAT, err);
		LOG_ERR("%s", at_req_resp_buf);
	}

	/* Free the old container object and create a new one to contain our response */
	cJSON_Delete(msg_obj);
	msg_obj = create_timestamped_device_message(
		NRF_CLOUD_JSON_APPID_VAL_MODEM,
		NRF_CLOUD_JSON_MSG_TYPE_VAL_DATA
	);

	if (!msg_obj) {
		return;
	}

	/* Populate response with command result */
	if (!cJSON_AddStringToObject(msg_obj, NRF_CLOUD_JSON_DATA_KEY, at_req_resp_buf)) {
		LOG_ERR("Failed to populate AT CMD response with modem response data");
		goto cleanup;
	}

	/* Send the response */
	err = send_device_message_cJSON(msg_obj);

	if (err) {
		LOG_ERR("Failed to send AT CMD request response, error: %d", err);
	}

cleanup:
	cJSON_Delete(msg_obj);
}

/** @brief Check whether temperature is acceptable.
 * If the device exceeds a temperature limit, send the temperature alert one time.
 * Once the temperature falls below a lower limit, re-enable the temperature alert
 * so it will be sent if limit is exceeded again.
 *
 * The difference between the two limits should be sufficient to prevent sending
 * new alerts if the temperature value oscillates between two nearby values.
 *
 * @param temp - The current device temperature.
 */
static void monitor_temperature(double temp)
{
	static bool temperature_alert_active;

	if ((temp > TEMP_ALERT_LIMIT) && !temperature_alert_active) {
		temperature_alert_active = true;
	//	(void)nrf_cloud_alert_send(ALERT_TYPE_TEMPERATURE, (float)temp,
	//				   "Temperature over limit!");
	} else if (temp < TEMP_ALERT_LOWER_LIMIT) {
		temperature_alert_active = false;
	}
}


void main_application_thread_fn(void)
{
 /*
    const struct device *motPos;
	motPos = device_get_binding(RPI_GPIO_LABEL);
	if (motPos == NULL) {
		LOG_INF("Error: didn't find %s device\n", RPI_GPIO_LABEL);
		LOG_INF("Error");
		return;
	}
        int ret;
	ret = gpio_pin_configure(motPos, RPI_GPIO_PIN, GPIO_OUTPUT_ACTIVE | RPI_GPIO_FLAGS);
	if (ret != 0) {
		LOG_INF("Error %d: failed to configure %s pin %d\n",
		       ret, RPI_GPIO_LABEL, RPI_GPIO_PIN);
		LOG_INF("Error");
		return;
	}
*/
/*
    const struct device *devGpio;
    bool led_is_on = true;
    int ret;
    dev = DEVICE_DT_GET(DT_NODELABEL(gpiocus0));

    devGpio = device_get_binding(gpiocus0);
    if (devGpio == NULL) {
		LOG_INF("Not configured1");
    }
    else 
	{
		LOG_INF("Periphal %s initialised, status %s \n", "Gpio00", GPIO0_STATUS);
	}
    ret = gpio_pin_configure(devGpio, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    if (ret < 0) {
		LOG_INF("Not configured2");
    }
*/

	cameraCtr.cameraFrequency = CONFIG_CAMERA_TIMER;
	int success = gpio_pin_configure(
    gpio_control.solarChargerEnable.port,
    gpio_control.solarChargerEnable.pin,
    gpio_control.solarChargerEnable.flags
  );

  	int success2 = gpio_pin_configure(
    gpio_control.rpiChargerEnable.port,
    gpio_control.rpiChargerEnable.pin,
    gpio_control.rpiChargerEnable.flags
  );

  	int success3 = gpio_pin_configure(
    gpio_control.solarChargerStatus.port,
    gpio_control.solarChargerStatus.pin,
    gpio_control.solarChargerStatus.flags
  );

    	int success4 = gpio_pin_configure(
    gpio_control.rpiChargerLow.port,
    gpio_control.rpiChargerLow.pin,
    gpio_control.rpiChargerLow.flags
  );

    	int success5 = gpio_pin_configure(
    gpio_control.solarChargerPGood.port,
    gpio_control.solarChargerPGood.pin,
    gpio_control.solarChargerPGood.flags
  );

      	int success6 = gpio_pin_configure(
    gpio_control.icarusCharger.port,
    gpio_control.icarusCharger.pin,
    gpio_control.icarusCharger.flags
  );

        	int success7 = gpio_pin_configure(
    gpio_control.rpiShutdown.port,
    gpio_control.rpiShutdown.pin,
    gpio_control.rpiShutdown.flags
  );

 // gpio_pin_set(
 //   three_digit.d1_en.port,
  //  three_digit.d1_en.pin,
  //  0   // or 1..
//);
	if (IS_ENABLED(CONFIG_AT_CMD_REQUESTS)) {
		/* Register with connection.c to receive general device messages and check them for
		 * AT command requests.
		 */
		register_general_dev_msg_handler(handle_at_cmd_requests);
	}
	int err;
	int err2;
	/* Wait for first connection before starting the application. */
	(void)await_connection(K_FOREVER);

//	(void)nrf_cloud_alert_send(ALERT_TYPE_DEVICE_NOW_ONLINE, 0, NULL);

	/* Wait for the date and time to become known.
	 * This is needed both for location services and for sensor sample timestamping.
	 */
	LOG_INF("Waiting for modem to determine current date and time");
	if (!await_date_time_known(K_SECONDS(CONFIG_DATE_TIME_ESTABLISHMENT_TIMEOUT_SECONDS))) {
		LOG_WRN("Failed to determine valid date time. Proceeding anyways");
	} else {
		LOG_INF("Current date and time determined");
	}

	/* Begin tracking location at the configured interval. */
	(void)start_location_tracking(on_location_update,
					CONFIG_LOCATION_TRACKING_SAMPLE_INTERVAL_SECONDS);
	init_adc();

	int counter = 0;
	k_work_init_delayable(&server_transmission_work,
			      server_transmission_work_fn);
	k_work_init(&uart_work,
			      uart_work_fn);

	k_work_init_delayable(&rpi_power_off_work,
			      rpi_power_off_work_fn);
	err = server_init();
	if (err) {
		LOG_ERR("Not able to initialize UDP server connection\n");
	//	return;
	}

	err = server_connect();
	if (err) {
		LOG_ERR("Not able to connect to UDP server\n");
	//	return;
	}
	err2 = server_init_stat();
	if (err2) {
		LOG_ERR("Not able to initialize UDP server connection\n");
	//	return;
	}

	err2 = server_connect_stat();
	if (err2) {
		LOG_ERR("Not able to connect to UDP server\n");
	//	return;
	}


	//ble_init();
	if (!device_is_ready(uart_dev)) {
		LOG_INF("UART device not found!");
	//	return;
	}

	/* configure interrupt and callback to receive data */
	uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	uart_irq_rx_enable(uart_dev);
//cc	int charging = adp536x_charging_enable(false);
//	LOG_INF("Starting SHA256 example...");
//	int status;
//	status = crypto_init();
//	if (status != APP_SUCCESS) {
//		LOG_INF(APP_ERROR_MESSAGE);
		//	return APP_ERROR;
//	}
	cameraCtr.takePictureNow = true;
	k_work_schedule(&server_transmission_work, K_NO_WAIT);
	k_work_schedule(&uart_work, K_NO_WAIT);
//	k_work_schedule(&rpi_power_off_work, K_FOREVER);

	static const struct device *gpio_dev;
	gpio_dev = DEVICE_DT_GET(GPIO_NODE);

	//static struct gpio_dt_spec red_led = GPIO_DT_SPEC_GET(RED_LED_NODE, gpios);


//	if (!device_is_ready(dev)) {
//		printk("sensor: device not ready.\n");
//		return;
//	}

	/* Begin sampling sensors. */
	while (true) {
		/* Start the sensor sample interval timer.
		 * We use a timer here instead of merely sleeping the thread, because the
		 * application thread can be preempted by other threads performing long tasks
		 * (such as periodic location acquisition), and we want to account for these
		 * delays when metering the sample send rate.
		 */

		uint16_t battery_voltage = 0;
		get_battery_voltage(&battery_voltage);
		LOG_INF("Battery voltage: %u mV\n", battery_voltage);
		bStatus.batteryVoltage = battery_voltage;
		int err = modem_info_short_get(MODEM_INFO_BATTERY, &bat_voltage);
		bStatus.modemBatteryVoltage = bat_voltage;
		if (err != sizeof(bat_voltage)) { 
			LOG_ERR("modem_info_short_get, error: %d", err);
	//		return err;
		}

	//	int batteryVolts = adp536x_fg_volts();
	//	int batteryPercentage = adp536x_fg_soc();
		k_timer_start(&sensor_sample_timer,
			K_SECONDS(CONFIG_SENSOR_SAMPLE_INTERVAL_SECONDS), K_FOREVER);

		if (IS_ENABLED(CONFIG_TEMP_TRACKING)) {
			double temp = -1;

			if (get_temperature(&temp) == 0) {
				LOG_INF("Temperature is %d degrees C", (int)temp);
				(void)send_sensor_sample(NRF_CLOUD_JSON_APPID_VAL_TEMP, temp);
			//	strncpy(newMess.temperatureLocal, temp,sizeof(temp));
				monitor_temperature(temp);
			}
		}

	//	if (IS_ENABLED(CONFIG_TEST_COUNTER)) {
	//		(void)send_sensor_sample("COUNT", counter++);
	//	}

		/* Wait out any remaining time on the sample interval timer. */
		k_timer_status_sync(&sensor_sample_timer);
	}
}



