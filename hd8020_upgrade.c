#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <sys/un.h>
#include <string.h>
#include <sys/prctl.h>
#include <errno.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/kdev_t.h>
#include <getopt.h>

#define SUCCESS 0
#define FAIL -1
#define UNKNOWN_CMD -2

#define PORT_NAME "/dev/ttyHSL1"
#define FILE_NAME "/system/etc/firmware/hd8020_bdfw.bin"
#define READ_LENGTH 200

#define HD8020_PWR_CTL "/sys/class/hd8020_bdgps_power/hd8020_bdgps/ctl_hd8020"
#define HD8020_FORCE_BOOT_CTL "/sys/class/hd8020_bdgps_power/hd8020_bdgps/force_to_bootmode"
#define HD8020_CTL_OFF   "0"
#define HD8020_CTL_ON    "1"

static int uart_fd = -1;
static int fw_fd = -1;
static int debug_on = 1;
static int power_flag = 0;

typedef unsigned char      uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int       uint32_t;

int uart_flush_input(int fd);
int uart_send(int fd, char *cmd, int cmd_len);
int uart_recv(int fd, char *buf, int *actual_length, int timeout);
int send_cmd_and_recv_result(char *cmd, char *recv_buf, int send_length, int *recv_length, int timeout);
const uint8_t cmdMONVER[] = {0xf1, 0xd9, 0x0a, 0x04, 0x00, 0x00, 0x0e, 0x34};
const uint8_t cmdSETFRQ[] = {0xf1, 0xd9, 0xf4, 0x00, 0x04, 0x00, 0x80, 0xba, 0x8c, 0x01, 0xbf, 0xff};
const uint8_t cmdCFGFWUP[] = {0xf1, 0xd9, 0x06, 0x50, 0x01, 0x00, 0x10, 0x67, 0x71, 0x00};
const uint8_t cmdBOOTERASE[] = {0xf1, 0xd9, 0xf4, 0x05, 0x06, 0x00, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x8f, 0x95};
const uint8_t cmdBOOTERASEALL[] = {0xf1, 0xd9, 0xf4, 0x08, 0x04, 0x00, 0xFF, 0xFF, 0xFF, 0x90, 0x8D, 0x77};
const uint8_t cmdCFGRST[] = {0xf1, 0xd9, 0x06, 0x40, 0x01, 0x00, 0x00, 0x47, 0x21};
const uint8_t cmdBOOTBAUD[] = {0xf1, 0xd9, 0xf4, 0x03, 0x08, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x85, 0x7d};
#define fw_uart_send(data, len) uart_send(uart_fd, data, len)
#define fw_uart_sendbyte(data) uart_send(uart_fd, data, 1)
#define fw_uart_receive(data, len) uart_recv(uart_fd, data, &len, 4)

uint8_t cmdNMEAOFF[] = {
	0xf1, 0xd9, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x00, 0x00, 0xfa, 0x0f,
	0xf1, 0xd9, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x02, 0x00, 0xfc, 0x13,
	0xf1, 0xd9, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x03, 0x00, 0xfd, 0x15,
	0xf1, 0xd9, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x04, 0x00, 0xfe, 0x17,
	0xf1, 0xd9, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x05, 0x00, 0xff, 0x19,
	0xf1, 0xd9, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x06, 0x00, 0x00, 0x1b,
	0xf1, 0xd9, 0x06, 0x01, 0x03, 0x00, 0xf0, 0x07, 0x00, 0x01, 0x1d
};

uint8_t data1st[] = {
	0x67, 0x6e, 0x73, 0x73, 0x32, 0x36, 0x72, 0x65, 0x6c, 0x31, 0x36, 0x2e, 0x67, 0x7a, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const uint8_t datalast[128] = { 0 };
const uint8_t cmdACKACK[] = {0xf1, 0xd9, 0x06, 0x40, 0x01, 0x00, 0x00, 0x47, 0x21};
const uint8_t cmdCFGPRT[] = {0xf1, 0xd9, 0x06, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0xd1, 0xe0};
uint8_t dataLast1K[1024] = { 0 };
unsigned char file_data[0x4B000] = { 0 };

/**
 * @brief  Boot mode send one block data of firmware
 * @param  addr: address of the data in HD8030 chip memory
 * @param  cnt : the number of the block
 * @param  data: the pointer of the block data
 * @param  len : the length of data
 * @retval None
 */
void sendfwboot(uint32_t addr, uint16_t cnt, uint8_t *data, uint32_t len)
{
	uint8_t cmd[] = {0xf1, 0xd9, 0xf4, 0x05, 0x06, 0x04, 0x00, 0x04, 0x00, 0x90, 0x01, 0x00};
	uint8_t ck[2] = { 0 };
	uint32_t i;
	uint32_t ck1 = 0, ck2 = 0;
	int actual_length = 0;
	char recv_buf[READ_LENGTH] = {0};

	*((uint16_t *)(cmd + 4)) = len + 6;
	*((uint32_t *)(cmd + 6)) = addr;
	*((uint16_t *)(cmd + 10)) = cnt;
	//check sum
	for(i = 0; i < 12; ++i)
	{
		if(i >= 2)
		{
			ck1 += cmd[i];
			ck2 += ck1;
		}
	}
	for(i = 0; i < len; ++i)
	{
		ck1 += data[i];
		ck2 += ck1;
	}
	ck[0] = ck1;
	ck[1] = ck2;

	uart_send(uart_fd, (uint8_t *)cmd, 12);
	uart_send(uart_fd, (uint8_t *)data, len);

	send_cmd_and_recv_result((uint8_t *)ck, recv_buf, 2, &actual_length, 3);
}

/**
 * @brief  calculate the CRC
 * @param  ptr  : the pointer of the block data
 * @param  count: the length of data
 * @retval the CRC result
 */
uint16_t calcrc(uint8_t *ptr, int count)
{
	uint32_t crc, i;

	crc = 0;

	while(--count >= 0)
	{
		crc = crc ^ (int)*ptr++ << 8;
		for (i = 0; i < 8; ++i)
			if (crc & 0x8000)
				crc = crc << 1 ^ 0x1021;
			else
				crc = crc << 1;
	}

	return(crc  & 0xFFFF);
}

/**
 * @brief  User mode send one block data of firmware
 * @param  cmd : send data command
 * @param  cnt : the number of the block
 * @param  data: the pointer of the block data
 * @param  len : the length of data
 * @retval None
 */
void sendfw(uint8_t cmd, uint8_t cnt, uint8_t *data, uint32_t len)
{
	uint16_t crc16;
	uint8_t cmd_ymodem,cnt_ymodem,cnt_ymodem_left,crc_temp[2];
	cmd_ymodem = cmd;
	cnt_ymodem = cnt;

	crc16 = calcrc(data, len);
	crc_temp[0] = crc16 >> 8;
	crc_temp[1] = crc16 & 0xFF;

	uart_send(uart_fd, &cmd_ymodem, 1);
	uart_send(uart_fd, &cnt_ymodem, 1);
	cnt_ymodem_left = 0xff - cnt_ymodem;
	uart_send(uart_fd, &cnt_ymodem_left, 1);
	uart_send(uart_fd, data, len);
	uart_send(uart_fd, &crc_temp[0], 1);
	uart_send(uart_fd, &crc_temp[1], 1);
}

/**
 * @brief  Boot mode firmware update
 * @param  data: the pointer of the firmware data
 * @param  len : the length of firmware data
 * @retval None
 */
void fw_update_boot(uint8_t *data, uint32_t len)
{
	uint16_t i = 1;
	uint8_t dataAck[10];
	//1st 1KB send at last
	uint8_t *data1st1k = data;
	data += 1024;
	len -= 1024;

	//config
	uart_send(uart_fd,(uint8_t *)cmdMONVER, 8);
	usleep(5000);//delay 5ms
	uart_send(uart_fd,(uint8_t *)cmdSETFRQ, 12);
	usleep(8000);//delay 8ms
	uart_send(uart_fd,(uint8_t *)cmdBOOTERASEALL, 12);
	usleep(800000);//delay 800ms
	uart_send(uart_fd,(uint8_t *)cmdBOOTERASE, 14);
	usleep(500000);//delay 170ms

	for(i = 1 ;len > 1024; ++i, data += 1024, len -= 1024)
	{
		sendfwboot(0x90000000 + i * 0x400, i, data, 1024);
	}
	sendfwboot(0x90000000 + i * 0x400, i, data, len);
	i++;

	sendfwboot(0x90000000, i, data1st1k, 1024);
	usleep(10000);
	uart_send(uart_fd, (uint8_t *)cmdCFGRST, 9);
	uart_send(uart_fd, (uint8_t *)cmdBOOTBAUD, 16);
}

/**
 * @brief  User mode firmware update
 * @param  data: the pointer of the firmware data
 * @param  len : the length of firmware data
 * @retval None
 */
void fw_update_user(uint8_t *data, uint32_t len)
{
	uint8_t rdata[20], tmpi;
	uint8_t i = 0;
	int actual_length = 0;
	uint8_t ymoden_end;

	//fill the fw length into data1st
	sprintf((char *)(data1st + 15), "%d", len);

	//send fwup conmmand
	uart_send(uart_fd, (uint8_t *)cmdCFGFWUP, 10);
	usleep(100000);

	uart_flush_input(uart_fd);
	uart_recv(uart_fd, rdata, &actual_length, 4);

	if(debug_on == 1)
		printf("rcv data %d %d %d %d %d\n",rdata[0],rdata[1],rdata[2],rdata[3],rdata[4]);

	if(rdata[0] != 'C')
	{
		printf("rcv data != C :%d\n", 'C');
		return;
	}

	//send 1st 133 data
	sendfw(0x01, i, (uint8_t *)data1st, 128);
	i++;

	uart_recv(uart_fd, rdata, &actual_length, 4);

	for(;len > 1024; ++i, data += 1024, len -= 1024)
	{
		sendfw(0x02, i, data, 1024);
		uart_recv(uart_fd, rdata, &actual_length, 4);
	}

	memcpy(dataLast1K, data, len);
	memset((dataLast1K + len), 0x1a, 1024 - len);
	sendfw(0x02, i, dataLast1K, 1024);
	uart_recv(uart_fd, rdata, &actual_length, 4);
	ymoden_end = 0x04;
	uart_send(uart_fd, &ymoden_end, 1);
	sendfw(0x01, 0, (uint8_t *)datalast, 128);
	uart_recv(uart_fd, rdata, &actual_length, 4);
	uart_send(uart_fd, (uint8_t *)cmdACKACK, 9);
	uart_send(uart_fd, (uint8_t *)cmdCFGPRT, 16);
	uart_send(uart_fd, (uint8_t *)cmdMONVER, 8);
}

int open_uart()
{
	int ret = SUCCESS;
	struct termios gps_termios;

	uart_fd = open(PORT_NAME, O_RDWR | O_NOCTTY | O_TRUNC | O_NONBLOCK  );
	if (uart_fd < 0)
	{
		printf("Could not open uart port");
		ret = FAIL;
		return ret;
	}

	if (tcgetattr(uart_fd, &gps_termios) != 0)
	{
		printf("tcgetattr(%d)\r\n", uart_fd);
		ret = FAIL;
		close(uart_fd);
		return ret;
	}

	gps_termios.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
	gps_termios.c_oflag &= ~OPOST;
	gps_termios.c_lflag &= ~(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
	gps_termios.c_cflag &= ~(CSIZE|PARENB);
	gps_termios.c_cflag |= CS8;
	gps_termios.c_cflag &= ~CRTSCTS;//no flow control

	tcsetattr(uart_fd, TCSANOW, &gps_termios);
	tcflush(uart_fd, TCIOFLUSH);
	tcsetattr(uart_fd, TCSANOW, &gps_termios);
	tcflush(uart_fd, TCIOFLUSH);
	tcflush(uart_fd, TCIOFLUSH);

	cfsetospeed(&gps_termios, B115200);
	cfsetispeed(&gps_termios, B115200);
	tcsetattr(uart_fd,TCSANOW, &gps_termios);

	return ret;
}

int uart_send(int fd, char *cmd, int cmd_len)
{
	int ret = SUCCESS;
	//TODO:add mutex & move open to init

	//send
	ret = write(fd, cmd, cmd_len);
	if(ret < 0)
	{
		printf("write port failed,ret: %d", ret);
		return FAIL;
	}
	if(debug_on == 1){
		printf("in %s, write %d \n", __func__, ret);
	}
	return SUCCESS;
}

int uart_flush_input(fd)
{
	if (tcflush(fd, TCIFLUSH) <0) {
		printf("Could not flush uart port");
		return -1;
	}

	return 0;
}

int uart_recv(int fd, char *buf, int *actual_length, int timeout)
{
	int ret;
	fd_set rd;
	struct timeval tv;
	tv.tv_sec = timeout;
	tv.tv_usec = 0;
	int i = 0;

	FD_ZERO(&rd);
	FD_SET(fd, &rd);
	ret = select(fd + 1, &rd, NULL, NULL, &tv);
	if(ret > 0){
		if (FD_ISSET(fd, &rd)) {
			*actual_length = read(fd, buf, READ_LENGTH);
			if(debug_on == 1){
				printf("Get the %d bytes: \n", *actual_length);
				for(i=0; i< *actual_length; i++)
				{
					printf("%x: ", *(buf+i));
				}
				printf("\n");

			}
		}
	}else if(ret == 0){
		printf("%s:timeout\n",__func__);
		goto ERR1;
	}else{
		printf("in %s, select error!\n", __func__);
		goto ERR1;
	}
	return 0;

ERR1:
	return FAIL;
}

int send_cmd_and_recv_result(char *cmd, char *recv_buf, int send_length, int *recv_length, int timeout)
{
	int ret = SUCCESS;

	/* printf("uart_fd:%d, send_length:%d\n",uart_fd, send_length); */
	ret = uart_send(uart_fd, cmd, send_length);
	if(ret != 0)
	{
		printf("in %s, uart send %s failed\n",__func__, cmd);
		return  ret;
	}

	uart_flush_input(uart_fd);

	ret = uart_recv(uart_fd, recv_buf, recv_length, timeout);
	if(ret != 0)
	{
		printf("in %s, uart recv %s failed\n",__func__, recv_buf);
		return  ret;
	}

	return SUCCESS;
}

int disable_nmea_output(void)
{
#if 1
	int ret = SUCCESS;

	ret = uart_send(uart_fd, cmdNMEAOFF, 77);
	if(ret)
	{
		printf("in %s, data transfer failed: %d \n",__func__, ret);
		return FAIL;
	}
#else
	int i = 0;
	int ret = SUCCESS;
	int actual_length = 0;
	char recv_buf[READ_LENGTH] = {0};

	ret = send_cmd_and_recv_result(cmdNMEAOFF, recv_buf, 77, &actual_length, 3);
	if(ret)
	{
		printf("in %s, data transfer failed: %d \n",__func__, ret);
		return FAIL;
	}

	for(i=0; i < actual_length; i++){
		printf("%x ", recv_buf[i]);
	}
	printf("\n");
#endif
	return ret;
}

int get_mon_ver(char *sw_ver, char *hw_ver)
{
	int i = 0;
	int ret = SUCCESS;
	int actual_length = 0;
	char recv_buf[READ_LENGTH] = {0};

	ret = disable_nmea_output();
	if (ret < 0)
	{
		printf("disable nmea output failed...ret:%d\n", ret);
		return FAIL;
	}
	usleep(200000);

	ret = send_cmd_and_recv_result(cmdMONVER, recv_buf, 8, &actual_length, 3);
	if(ret)
	{
		printf("in %s, data transfer failed: %d \n",__func__, ret);
		return FAIL;
	}

	if(debug_on == 1)
	{
		for(i=0; i < actual_length; i++){
			printf("%x ", recv_buf[i]);
		}
		printf("actual_length:%d\n", actual_length);
		printf("sw_ver:%s\n", recv_buf + 6);
		printf("hw_ver:%s\n", recv_buf + 22);
	}
	memcpy(sw_ver, recv_buf + 6, 16);
	memcpy(hw_ver, recv_buf + 22, 16);

	return ret;
}

int open_fw_file(char *fw_path)
{
	int fw_len;
	int ret = 0;

	fw_fd = open(fw_path, O_RDONLY);
	if (fw_fd < 0)
	{
		printf("Could not open file %s\n", FILE_NAME);
		return FAIL;
	}

	fw_len = lseek(fw_fd, 0L, SEEK_END);
	if (fw_len <= 0)
	{
		printf("failed get file length, %d, errno: %s\n", fw_len, strerror(errno));
		close(fw_fd);
		return FAIL;
	}
	lseek(fw_fd, 0L, SEEK_SET);

	memset(file_data, 0, sizeof(file_data));

	ret = read(fw_fd, file_data, fw_len);
	if(ret < 0)
	{
		printf("read firmware file failed, ret:%d\n", ret);
		return FAIL;
	}

	return fw_len;
}

int check_flash_mode(int *boot_flag)
{
	int ret = SUCCESS;
	unsigned char sw_ver_tmp[20] = {0};
	unsigned char hw_ver_tmp[20] = {0};

	ret = get_mon_ver(sw_ver_tmp, hw_ver_tmp);
	if (ret < 0)
	{
		printf("disable nmea output failed...ret:%d\n", ret);
		return FAIL;
	}
	if(debug_on == 1){
		printf("sw_ver:%s\n", sw_ver_tmp);
		printf("hw_ver:%s\n", hw_ver_tmp);
	}
	if(strncmp(sw_ver_tmp, "FB02BOOT", 8) != 0){
		*boot_flag = 0;
	}else{
		*boot_flag = 1;
	}

	return ret;
}

int hd8020_power_ctl(int enable)
{
	int ret;
	int pwr_ctl_fd;

	if( (power_flag && enable) || (!power_flag && !enable))
		return 0;

	pwr_ctl_fd = open(HD8020_PWR_CTL, O_RDWR);
	if(pwr_ctl_fd < 0)
	{
		printf("open %s failed!\n", HD8020_PWR_CTL);
		return FAIL;
	}
	if(enable)
	{
		ret = write(pwr_ctl_fd, HD8020_CTL_ON, 1);
		if(ret < 0)
		{
			printf("write %s failed!\n", HD8020_PWR_CTL);
			close(pwr_ctl_fd);
			return FAIL;
		}
		power_flag = 1;
	}
	else
	{
		ret = write(pwr_ctl_fd, HD8020_CTL_OFF, 1);
		if(ret < 0)
		{
			printf("write %s failed!\n", HD8020_PWR_CTL);
			close(pwr_ctl_fd);
			return FAIL;
		}
		power_flag = 0;
	}
	close(pwr_ctl_fd);

	return SUCCESS;
}

int hd8020_force_boot_ctl(int enable)
{
	int ret;
	int pwr_ctl_fd;

	pwr_ctl_fd = open(HD8020_FORCE_BOOT_CTL, O_RDWR);
	if(pwr_ctl_fd < 0)
	{
		printf("open %s failed!\n", HD8020_FORCE_BOOT_CTL);
		return FAIL;
	}
	if(enable)
	{
		ret = write(pwr_ctl_fd, HD8020_CTL_ON, 1);
		if(ret < 0)
		{
			printf("write %s failed!\n", HD8020_FORCE_BOOT_CTL);
			close(pwr_ctl_fd);
			return FAIL;
		}
	}
	else
	{
		ret = write(pwr_ctl_fd, HD8020_CTL_OFF, 1);
		if(ret < 0)
		{
			printf("write %s failed!\n", HD8020_FORCE_BOOT_CTL);
			close(pwr_ctl_fd);
			return FAIL;
		}
	}
	close(pwr_ctl_fd);

	return SUCCESS;
}

int upgrade_firmware_user()
{
	int ret = FAIL;
	int boot_flag = 0;
	int fw_len;

	printf("upgrade with user mode...\n");
	//open firmware bin file
	fw_len = open_fw_file(FILE_NAME);
	if(fw_len <= 0)
	{
		printf("open_fw_file failed..., ret:%d\n", ret);
		return FAIL;
	}
	else
	{
		printf("fw file size is %d\n", fw_len);
	}

	//power up
	ret = hd8020_power_ctl(1);
	if(ret < 0)
	{
		printf("hd8020 power up failed..., ret:%d\n", ret);
		return FAIL;
	}

	//check flash mode
	ret = check_flash_mode(&boot_flag);
	if(ret < 0)
	{
		printf("check flash mode failed..., ret:%d\n", ret);
		return FAIL;
	}
	else
	{
		printf("get boot_flag: %d\n", boot_flag);
	}

	fw_update_user(file_data, fw_len);

	//power off
	ret = hd8020_power_ctl(0);
	if(ret < 0)
	{
		printf("hd8020 power off failed..., ret:%d\n", ret);
		return FAIL;
	}

	//check upgrade status
	return SUCCESS;
}

int upgrade_firmware_boot()
{
	int ret = FAIL;
	int boot_flag = 0;
	int fw_len;
	printf("upgrade with boot mode...\n");

	//open firmware bin file
	fw_len = open_fw_file(FILE_NAME);
	if(fw_len <= 0)
	{
		printf("open_fw_file failed..., ret:%d\n", ret);
		return FAIL;
	}
	else
	{
		printf("fw file size is %d\n", fw_len);
	}

	//force to boot mode in
	ret = hd8020_force_boot_ctl(1);
	if(ret < 0)
	{
		printf("hd8020 power up failed..., ret:%d\n", ret);
		return FAIL;
	}

	//check flash mode
	ret = check_flash_mode(&boot_flag);
	if(ret < 0)
	{
		printf("check flash mode failed..., ret:%d\n", ret);
		return FAIL;
	}

	fw_update_boot(file_data, fw_len);

	//force to boot mode out
	ret = hd8020_force_boot_ctl(0);
	if(ret < 0)
	{
		printf("hd8020 power off failed..., ret:%d\n", ret);
		return FAIL;
	}

	//check upgrade status
	return SUCCESS;
}

int upgrade_firmware_auto()
{
	int ret = FAIL;
	int boot_flag = 0;
	int fw_len;
	printf("upgrade with auto mode...\n");

	//open firmware bin file
	fw_len = open_fw_file(FILE_NAME);
	if(fw_len <= 0)
	{
		printf("open_fw_file failed..., ret:%d\n", ret);
		return FAIL;
	}
	else
	{
		printf("fw file size is %d\n", fw_len);
	}

	//power up
	ret = hd8020_power_ctl(1);
	if(ret < 0)
	{
		printf("hd8020 power up failed..., ret:%d\n", ret);
		return FAIL;
	}

	//check flash mode
	ret = check_flash_mode(&boot_flag);
	if(ret < 0)
	{
		printf("check flash mode failed..., ret:%d\n", ret);
		return FAIL;
	}
	else
	{
		printf("get boot_flag: %d\n", boot_flag);
	}

	//upgrade firmware
	if(boot_flag){
		fw_update_boot(file_data, fw_len);
	}else{
		fw_update_user(file_data, fw_len);
	}

	//power off
	ret = hd8020_power_ctl(0);
	if(ret < 0)
	{
		printf("hd8020 power off failed..., ret:%d\n", ret);
		return FAIL;
	}

	//check upgrade status
	return SUCCESS;
}

int set_message(unsigned char* dst,unsigned char* src,int src_len)
{
	int i;

	//message header
	dst[0]=0xF1;
	dst[1]=0xD9;
	//message ID
	dst[2]=0x06;
	dst[3]=0x40;
	//message len
	dst[4]=src_len;
	dst[5]=00;

	//payload
	for( i = 0 ; i < src_len ; i++)
	{
		dst[6+i] = *src ;
		src++ ;
	}

	unsigned char checksum1 = 0x00;
	unsigned char checksum2 = 0x00;
	for(i=2;i<(6+dst[4]);i++){
		//printf("%2X ",dst[i]);
		checksum1+=dst[i];
		checksum2+=checksum1;
	}

	dst[src_len+6]= checksum1;
	dst[src_len+7]= checksum2;
	dst[src_len+8]=0x0d;
	dst[src_len+9]=0x0a;

	return src_len+10 ;
}

int cold_start(int fd_global){
	unsigned char cmd[1],message[11];
	int len;
	int ret;

	memset(cmd,0,sizeof(cmd));
	cmd[0] = 0x01;
	len = set_message(message, cmd, sizeof(cmd));
	if(debug_on == 1){
		printf("in %s, message: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", __func__,
				message[0],message[1],message[2],message[3],message[4],message[5],message[6],message[7],
				message[8],message[9],message[10]);
	}
	ret = write(fd_global,message,len);
	if(debug_on == 1){
		printf("in %s, ret:%d, len:%d\n", __func__, ret, len);
	}
	return 1;
}

int hot_start(int fd_global){
	unsigned char cmd[1],message[11];
	int len;
	int ret;

	memset(cmd,0,sizeof(cmd));
	cmd[0] = 0x03;
	len = set_message(message,cmd,sizeof(cmd));
	if(debug_on == 1){
		printf("in %s, message: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", __func__,
				message[0],message[1],message[2],message[3],message[4],message[5],message[6],message[7],
				message[8],message[9],message[10]);
	}
	ret = write(fd_global,message,len);
	if(debug_on == 1){
		printf("in %s, ret:%d, len:%d\n", __func__, ret, len);
	}
	return 1;
}

int warm_start(int fd_global){
	unsigned char cmd[1],message[11];
	int len;
	int ret;

	memset(cmd,0,sizeof(cmd));
	cmd[0] = 0x02;
	len = set_message(message,cmd,sizeof(cmd));
	printf("in %s, message: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", __func__,
			message[0],message[1],message[2],message[3],message[4],message[5],message[6],message[7],
			message[8],message[9],message[10]);
	ret = write(fd_global,message,len);

	printf("in %s, ret:%d, len:%d\n", __func__, ret, len);
	return 1;
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-FBPVh]\n", prog);
	puts("  -F --flash   flash mode select (boot or user or auto) \n"
	     "  -B --boot    boot mode select (cold or hot or warm) \n"
	     "  -P --power   power control (on or off) \n"
	     "  -D --debug   debug control (on or off) \n"
	     "  -V --version firmware version get \n"
	     "  -h --help    help message print \n");
	exit(1);
}

int main(int argc, char **argv)
{
	int ret = 0;
	unsigned char sw1_ver[20] = {0};
	unsigned char hw1_ver[20] = {0};

	if (argc == 1) {
		printf("This program needs arguments....\n\n");
		print_usage(argv[0]);
	}

	ret = open_uart(PORT_NAME);
	if (ret < 0)
	{
		printf("open uart failed %d\n", ret);
		return -1;
	}

	if(debug_on == 1)
		printf("uart dev open successfully...fd:%d\n", uart_fd);

	while (1) {
		int c;
		static const struct option lopts[] = {
			{ "flash",  required_argument, NULL, 'F' },
			{ "boot",   required_argument, 0, 'B' },
			{ "power",   required_argument, 0, 'P' },
			{ "debug",   required_argument, 0, 'D' },
			{ "version",   0, 0, 'V' },
			{ "help",   0, 0, 'h' },
			{ NULL, 0, 0, 0 },
		};

		c = getopt_long(argc, argv, "F:B:P:D:Vh", lopts, NULL);
		if (c == -1)
			break;

		switch (c) {
		case 'F':
			if ( optarg == NULL )
			{
				printf("%s: option 'F' requires argument...\n", argv[0]);
				break;
			}
			/* printf("optarg:%s\n", optarg); */

			if (!(strcasecmp(optarg, "boot"))){
				upgrade_firmware_boot();
			}else if (!(strcasecmp(optarg, "user"))){
				upgrade_firmware_user();
			}else if (!(strcasecmp(optarg, "auto"))){
				upgrade_firmware_auto();
			}else{
				print_usage(argv[0]);
			}
			break;
		case 'B':
			if ( optarg == NULL )
			{
				printf("%s: option 'B' requires argument...\n", argv[0]);
				break;
			}
			hd8020_power_ctl(1);
			sleep(1);
			if (!(strcasecmp(optarg, "cold"))){
				cold_start(uart_fd);
			}else if (!(strcasecmp(optarg, "hot"))){
				hot_start(uart_fd);
			}else if (!(strcasecmp(optarg, "warm"))){
				warm_start(uart_fd);
			}else{
				print_usage(argv[0]);
			}
			sleep(1);
			hd8020_power_ctl(0);
			break;
		case 'P':
			if ( optarg == NULL )
			{
				printf("%s: option 'P' requires argument...\n", argv[0]);
				break;
			}
			if (!(strcasecmp(optarg, "on"))){
				hd8020_power_ctl(1);
			}else if (!(strcasecmp(optarg, "off"))){
				hd8020_power_ctl(0);
			}else{
				print_usage(argv[0]);
			}
			break;
		case 'D':
			if ( optarg == NULL )
			{
				printf("%s: option 'D' requires argument...\n", argv[0]);
				break;
			}
			if (!(strcasecmp(optarg, "on"))){
				debug_on = 1;
			}else if (!(strcasecmp(optarg, "off"))){
				debug_on = 0;
			}else{
				print_usage(argv[0]);
			}
			break;
		case 'V':
			memset(sw1_ver, '\0', sizeof(sw1_ver));
			memset(hw1_ver, '\0', sizeof(hw1_ver));
			hd8020_power_ctl(1);
			ret = get_mon_ver(sw1_ver, hw1_ver);
			if (ret < 0)
			{
				printf("get mon version failed...ret:%d\n", ret);
				return FAIL;
			}
			printf("sw1_ver:%s\n", sw1_ver);
			printf("hw1_ver:%s\n", hw1_ver);
			hd8020_power_ctl(0);
			break;
		case 'h':
			print_usage(argv[0]);
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}

	if(debug_on == 1)
		printf("close uart fd %d\n", uart_fd);

	uart_fd = -1;
	close(uart_fd);
	return ret;
}
