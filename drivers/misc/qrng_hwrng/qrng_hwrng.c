// ---------------------------------------------------------------------------
// vim: ts=4 sw=4:
// ---------------------------------------------------------------------------
//                             IDQuantique Co., Ltd.
//
//                 Copyright (C) 2019 IDQuantique Corporation.
//                          All rights reserved.
//
// Project  : QRNG
// Filename : qrng_hwrng.c
//
// ---------------------------------------------------------------------------
// Feature : hwrng driver for QRNG-IDQ250C2
// Function Description :
// Note:
// ---------------------------------------------------------------------------

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/hw_random.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/sched/signal.h>
#include <linux/io.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/mm.h>

#ifdef CONFIG_HW_RANDOM_IDQ250C2_MISC_DEVICE
#define	QRNG_IOCTL_DIAG			_IO('Q',4)
#endif

#define BURST_PKT_CNT 1
#define PKT_SIZE  18

static unsigned char *kbuf = NULL;

int qrng_get_data(char *data_buf, int data_size);
static int data_avail = 0;

#define CHUNK_SIZE	BURST_PKT_CNT * (PKT_SIZE -2)


#define QRNG_DEBUG

#ifdef  QRNG_DEBUG
#define PINFO(fmt, args...)  pr_info("[QRNG_I] %s(): " fmt, __FUNCTION__, ##args);
#define PDBG(fmt, args...)   pr_info("[QRNG_D] %s(): " fmt, __FUNCTION__, ##args);
#define PERR(fmt, args...)   pr_err ("[QRNG_E] %s(): " fmt, __FUNCTION__, ##args);
#define PLINE                pr_info("[QRNG_D] %s(): %d line\n", __FUNCTION__, __LINE__);
#else
#define PINFO(fmt, args...)  pr_info("[QRNG_I] %s(): " fmt, __FUNCTION__, ##args);
#define PDBG(fmt, args...)   do {} while(0);
#define PERR(fmt, args...)   pr_info("[QRNG_E] %s(): " fmt, __FUNCTION__, ##args);
#define PLINE                do {} while(0);
#endif

#define QRNG_PRO_WINSIZE		512
#define QRNG_PACKETSIZE			16
#define PKT_SIZE      			18    // 9 * 2 byte
#define QRNG_DEV_READY_FLAG   	0x01
#define QRNG_PRO_CUTOFF			15
#define QRNG_BIT_REP_H8			4

static uint8_t  gnQrngHchk = 0x00;
static uint32_t gnQrngHchkCnt = 0;

typedef enum _QRNG_ERR {
    QRNG_OK                 = 0,
    QRNG_NOT_EXIST        = 1,
    QRNG_HEADER_ERR       = 2,
    QRNG_INF_ERR          = 3,
    QRNG_VST_FAIL         = 4,
    QRNG_RETRY_TIMEOUT    = 5,
    QRNG_PARAM_ERROR      = 8,
    QRNG_CHIP_READ_ERROR  = 19,
    QRNG_BYTE_REP_ERROR   = 21,
    QRNG_PROPORTION_ERROR = 22,
    QRNG_ERROR            = 0xFFFF
} QRNG_ERR;

// ---------------------------------------------------------------------------
//END of IDQ250C2_LIB
// ---------------------------------------------------------------------------


typedef struct _QRNG_DEV {
  struct device     *dev;
  int               dev_num;
  char              dev_name[7];
  struct i2c_client *client;
  struct mutex      trans_mtx;
#ifdef CONFIG_QRNG_HQM
  QRNG_HQM          hqm;
#endif
  int               GPIO_PWDN;
  int               GPIO_LDO_EN;
  int               QRNG_LDO_EN_STATUS;
  int               QRNG_READY_STATUS;
  int               QRNG_PWDN_STATUS;
  int               QRNG_SUSPEND;
} QRNG_DEV;

#define	DEVICE_NAME		"qrng-i2c"

static QRNG_DEV *gQrng = NULL;

// ---------------------------------------------------------------------------
// hardware adapter function for qrng
// ---------------------------------------------------------------------------
static int __maybe_unused qrng_i2c_write(uint8_t nAddr, uint8_t nData)
{
	uint8_t		pBuff[2];

	PDBG("0x%02X:0x%02X", nAddr, nData);
	if (gQrng == NULL)	return -EIO;

	pBuff[0] = nAddr;
	pBuff[1] = nData;

	if (i2c_master_send(gQrng->client,pBuff,2) != 2)
	{
		PERR("i2c_master_send error\n");
		return -EIO;
	}

	return 0;
}

static int qrng_i2c_read(uint8_t nAddr, uint8_t *pData)
{
	if (gQrng == NULL)	return -EIO;

	if (i2c_master_send(gQrng->client,&nAddr,1) != 1)
	{
		PERR("i2c_master_send error : 0x%02X\n", nAddr);
		return -EIO;
	}
	if (i2c_master_recv(gQrng->client,pData,1) != 1)
	{
		PERR("i2c_master_recv error : 0x%02X\n", nAddr);
		return -EIO;
	}

	PDBG("0x%02X:0x%02X", nAddr, *pData);
	return 0;
}

static int qrng_i2c_read_burst(uint8_t nAddr, int nLen, uint8_t *pData)
{
	struct i2c_adapter	*adap = gQrng->client->adapter;
	struct i2c_msg		msgs[2];

	PDBG("0x%02X - length(%d)", nAddr, nLen);

	if (gQrng == NULL)	return -EIO;

	msgs[0].addr	= gQrng->client->addr;
	msgs[0].flags	= gQrng->client->flags;
	msgs[0].len		= 1;
	msgs[0].buf		= &nAddr;

	msgs[1].addr	= gQrng->client->addr;
	msgs[1].flags	= gQrng->client->flags | I2C_M_RD;
	msgs[1].len		= nLen;
	msgs[1].buf		= pData;

	if (i2c_transfer(adap, msgs, 2) != 2)
	{
		PERR("i2c_transfer error\n");
		return -EIO;
	}

	return 0;
}

static int qrng_get_vi(void)
{
	// vi status : 1 = ok / 0 = fail
	return 1;
}

static void qrng_delay_ms(int ms)
{
	if (ms >= 10)   msleep(ms);
	else            while (ms--)    udelay(1000);
}

static void* qrng_memcpy(void *dest,const void *src,unsigned int len)
{
	return memcpy(dest, src, len);
}


static int qrng_pwdn(int pol)
{
	int	ret;
	PDBG("%d", pol);
	if (pol)	// set pwdn high
	{
		ret = gpio_direction_output(gQrng->GPIO_PWDN, 1);
		if (!ret) {
			gQrng->QRNG_PWDN_STATUS = 1;
			PINFO("QRNG_PWDN_STATUS = 1");
		}
	}
	else
	{	// set pwdn low
		ret = gpio_direction_output(gQrng->GPIO_PWDN, 0);
		if (!ret) {
			gQrng->QRNG_PWDN_STATUS = 0;
			PINFO("QRNG_PWDN_STATUS = 0");
		}
	}
	return ret;
}

// ---------------------------------------------------------------------------
// Health Test variable
// ---------------------------------------------------------------------------

static uint8_t	g_bQrngWindowData[QRNG_PRO_WINSIZE] = {0,};		// Propotion Window Buffer
static uint8_t	g_bHist[256] = {0,};				// Propotion Hist Buffer
static int		g_iWindowDataCnt = 0;				// Propotion Window Count

static int 		g_iQrngByteReinit = 1;				// Repetition Init Count (Byte)
static uint8_t 	g_bLastData[16] = {0,};				// Repetition Byte Last Data Buffer

static int 		g_QrngError_cnt = 0;				// Health Error Count (5 times Repeats => Return err)

// ---------------------------------------------------------------------------
// qrng function
// ---------------------------------------------------------------------------

#define	RETRY	10

static int qrng_rd_pkt(uint8_t pPkt[PKT_SIZE])
{
	int		ret, retry;

	for (retry=RETRY; retry>0; retry--)
	{
		// read packet
		ret = qrng_i2c_read_burst(0x12, PKT_SIZE, pPkt);
		if (ret != 0) {
			PDBG("QRNG_INF_ERR");
			return QRNG_INF_ERR;
		}

		// check header
		if (pPkt[0] != 0xA2)		return QRNG_HEADER_ERR;

		// check ready (not duplicated packet)
		else if (pPkt[1] != 0x08)
		{
			break;
		}
	}

	if (retry <= 0) {
		WARN("[QRNG] %s : QRNG_RETRY_TIMEOUT", __func__);
		return QRNG_RETRY_TIMEOUT;
	}
	return QRNG_OK;
}

// Name : qrng_propotion
// Function : Proportion Test 512 Windows (Cutoff : 13)
static int qrng_propotion(const uint8_t *data, int m_nEditProValue)
{

	int i;
	int iMax = 0;
	uint8_t bWindowdata[QRNG_PACKETSIZE];

	qrng_memcpy(&bWindowdata[0], data, QRNG_PACKETSIZE);
	qrng_memcpy(&g_bQrngWindowData[g_iWindowDataCnt], &bWindowdata[0],QRNG_PACKETSIZE);

	for(i = 0; i < QRNG_PACKETSIZE; i++)
	{
		g_bHist[bWindowdata[i]] += 1;

		if (g_bHist[bWindowdata[i]] >= iMax)
		{
			iMax = g_bHist[bWindowdata[i]];
		}
		else
		{
			;
		}
	}

	if(iMax >= m_nEditProValue)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//------------------------------------------------------------
// Name : qrng_byteRepetition
// Function : BYTE Repetition Check (default : 4 Bytes)
//------------------------------------------------------------
static int qrng_byteRepetition(const uint8_t *data, int m_nEditRepH8)
{
	int iCntByte = 0;
	int i;
	int init = g_iQrngByteReinit;
	uint8_t bTemp = 0;
	uint8_t bWindowByte[QRNG_PACKETSIZE];		// MAX 15+ 16
	uint8_t bLastData[QRNG_PACKETSIZE];

	qrng_memcpy(bLastData, g_bLastData, m_nEditRepH8);

	if (g_iQrngByteReinit == 1)
	{
		;
	}
	else
	{
		bTemp = bLastData[0];
		for(i=1;i < m_nEditRepH8; i++)
		{

			if(bTemp == bLastData[i])
			{
				iCntByte += 1;
			}
			else
			{
				iCntByte = 0;
			}

			bTemp = bLastData[i];

		}
	}

	qrng_memcpy(&bWindowByte, data, QRNG_PACKETSIZE);

	for (i = 0; i < QRNG_PACKETSIZE ; i++)
	{

		if(init == 1)
		{
			bTemp = bWindowByte[i];
			init = 0;
		}
		else
		{
			if (bTemp == bWindowByte[i])
			{
				iCntByte += 1;
			}
			else
			{
				iCntByte = 0;
			}
			if (iCntByte == (m_nEditRepH8-1))
			{
				return 1;
			}

			bTemp = bWindowByte[i];
		}
	}

	return 0;
}

//------------------------------------------------------------
// Name : qrng_healthTest
// Function : include Proportion , Repetition Bytes, BIT
//------------------------------------------------------------

static int qrng_healthTest(const uint8_t *data)
{
	static int q_iProOverFlame = 0;
	static int q_iReByteOverFlame = 0;

	int i;
	uint8_t ucPacketData[QRNG_PACKETSIZE];

	qrng_memcpy(ucPacketData, data, QRNG_PACKETSIZE);

	q_iProOverFlame = qrng_propotion(ucPacketData, QRNG_PRO_CUTOFF);

	q_iReByteOverFlame = qrng_byteRepetition(ucPacketData, QRNG_BIT_REP_H8);

	if((q_iReByteOverFlame == 0) && (q_iProOverFlame == 0))
	{
		g_iQrngByteReinit = 0;

		if (g_iWindowDataCnt < (QRNG_PRO_WINSIZE - QRNG_PACKETSIZE))
		{
			g_iWindowDataCnt += QRNG_PACKETSIZE;
		}
		else
		{
			for(i = 0; i < QRNG_PACKETSIZE; i++)
			{
				g_bHist[g_bQrngWindowData[i]]--;
			}

			qrng_memcpy(&g_bQrngWindowData[0],&g_bQrngWindowData[QRNG_PACKETSIZE], QRNG_PRO_WINSIZE - QRNG_PACKETSIZE);

		}
		qrng_memcpy(&g_bLastData[0], &ucPacketData[QRNG_PACKETSIZE - QRNG_BIT_REP_H8], QRNG_BIT_REP_H8);

	}
	else
	{
		for(i = 0; i <QRNG_PACKETSIZE; i++)
		{
			g_bHist[g_bQrngWindowData[g_iWindowDataCnt + i]]--;
		}


		if(q_iReByteOverFlame == 1)
		{
			return QRNG_BYTE_REP_ERROR;
		}

		else if(q_iProOverFlame == 1)
		{
			return QRNG_PROPORTION_ERROR;
		}
		else
		{
			;
		}
	}

	return QRNG_OK;
}

// ---------------------------------------------------------------------------
// qrng api
// ---------------------------------------------------------------------------

static void qrng_init_hchk(void)
{
	gnQrngHchk = 0x00;
	gnQrngHchkCnt = 0;
}

static int qrng_init(void)
{
	uint8_t buf[1] = {0xFF};

	if(!gQrng | !(gQrng->QRNG_READY_STATUS & QRNG_DEV_READY_FLAG)) {
		PERR("QRNG DEV is not ready.");
		return QRNG_NOT_EXIST;
	}

	if(qrng_pwdn(1) < 0) {
		PERR("Failed to set qrng_pwdn : 1");
		return QRNG_INF_ERR;
	}
	qrng_delay_ms(3);

	// initialize health check info
	qrng_init_hchk();

	if(qrng_i2c_read(0x10, buf) < 0) {
		PERR("Failed to read status registry");
		return QRNG_INF_ERR;
	}

	if (buf[0] != 0x0) {
		PERR("ERR I2C Read [0x10] : 0x%02X", buf[0]);
		return QRNG_HEADER_ERR;
	}

	return QRNG_OK;
}

static int qrng_rd_samp(uint8_t *pData, int nLen, uint8_t *pHchk)
{
	uint8_t	pPkt[PKT_SIZE];
	int		ret, i, j, pkt_cnt, idx = 0;
	int	health_count = 0, header_count = 0;

	if (gQrng->QRNG_SUSPEND == 1)
	{
		return -EFAULT;
	}

	if (gQrng->QRNG_PWDN_STATUS != 1)
	{
		ret = qrng_init();
		if (ret) {
			PERR("qrng_init error: %d\n", ret);
			return -EFAULT;
		}
	}

	pkt_cnt = (((nLen+1) >> 1) + 7) >> 3;

	*pHchk = 0;		// health check

	for (i=pkt_cnt; i>0; i--)
	{
samp_retry:
		// check vi fail
		if (qrng_get_vi() == 0)
			return QRNG_VST_FAIL;

		// read packet
		ret = qrng_rd_pkt(pPkt);
		if (ret != QRNG_OK)
		{
			if ((ret == QRNG_HEADER_ERR) && (header_count < 5)) {
				header_count++;
				goto samp_retry;
			}else{
				return ret;
			}
		}

		if (pPkt[1]) // check Health data
		{
			PERR("Health data wrong : 0x%02X", pPkt[1]);
			gnQrngHchk |= pPkt[1];
			gnQrngHchkCnt++;
			if (health_count < 5) {
				health_count++;
				goto samp_retry;
			}else{
				*pHchk |= pPkt[1];
#ifdef QRNG_DEBUG
				panic("Health data wrong");
#endif
				return QRNG_PARAM_ERROR;
			}
		}
		ret = qrng_healthTest(&pPkt[2]);

		if(ret != 0)
		{
			PINFO("SP800-90B approved tests failure : %d", ret);
			g_QrngError_cnt++;
			if(g_QrngError_cnt == 5)
			{
#ifdef QRNG_DEBUG
				panic("SP800-90B approved tests failure");
#endif
				return ret;
			}
			i++;
		}
		else
		{
			g_QrngError_cnt = 0;
			for (j=2; j<PKT_SIZE; j++)
			{
				*(pData + idx) = pPkt[j];
				idx++;

			}
		}
	}
	return QRNG_OK;
}

// ---------------------------------------------------------------------------
// qrng hwrng driver
// ---------------------------------------------------------------------------

static int qrng_rng_read(struct hwrng *rng, void *buf, size_t max, bool wait)
{
	int		ret;

	if (gQrng == NULL)	return -ENODEV;

	ret = qrng_get_data(buf, max);
	if (ret)
	{
		PERR("fail to get data on hwrng:%d\n", ret);
		return -EIO;
	}

	PDBG("QRNG writes %zu bytes\n", max);


	return max;
}

static struct hwrng qrng_rng;

// ---------------------------------------------------------------------------
// Timer Function
// ---------------------------------------------------------------------------
#ifdef CONFIG_HW_RANDOM_IDQ250C2_TIMER
#define   TIME_STEP                       300 //msec
static struct timer_list  timer;

void kerneltimer_timeover(struct timer_list *pTimer)
{
	if(gQrng->QRNG_PWDN_STATUS != 0)
	{
		gQrng->QRNG_PWDN_STATUS = 0;
		if(qrng_pwdn(0) < 0)
		{
			PERR("Failed to set qrng_pwdn : 0");
		}
	}
	qrng_delay_ms(3);
}

static void kerneltimer_register(struct timer_list *pTimer, unsigned long timeover)
{
	timer_setup(pTimer,kerneltimer_timeover, 0);
	mod_timer(pTimer, jiffies + msecs_to_jiffies(timeover));
}

static void kerneltimer_exit(struct timer_list *pTimer)
{
	if(pTimer != NULL)
	{
		del_timer(pTimer);
	}
}
#endif
// ---------------------------------------------------------------------------
// misc driver
// ---------------------------------------------------------------------------

int qrng_get_data(char *data_buf, int data_size)
{
	int ret = 0;
	char hchk = 0;
	int len = 0;
	unsigned int read_data = 0;

	if(data_size <= 0) {
		PERR("wrong data size : %d\n", data_size);
		return -EINVAL;
	}
	mutex_lock(&gQrng->trans_mtx);

#ifdef CONFIG_HW_RANDOM_IDQ250C2_TIMER
	kerneltimer_exit(&timer);
#endif

	while(data_size){
		if(!data_avail){
			ret = qrng_rd_samp(kbuf, CHUNK_SIZE, &hchk);
			if(ret){
				if(hchk){
					PERR("qrng_rd_samp health erro:hchk%02Xh(ret:%d)\n", hchk, ret);
				}else{
					PERR("qrng_rd_samp error: %d\n", ret);
				}
				break;
			}

			data_avail = CHUNK_SIZE;
		}

		len = data_avail;
		if(len > data_size)
			len = data_size;

		data_avail -= len;

		qrng_memcpy(data_buf+read_data, kbuf+data_avail, len);

		data_size -= len;
		read_data += len;

	}

#ifdef CONFIG_HW_RANDOM_IDQ250C2_TIMER
	kerneltimer_register(&timer, TIME_STEP);
#endif
	mutex_unlock(&gQrng->trans_mtx);

	return ret;
}

EXPORT_SYMBOL(qrng_get_data);

#ifdef CONFIG_HW_RANDOM_IDQ250C2_MISC_DEVICE
static int qrng_op_open(struct inode *inode, struct file *filp)
{
	PLINE;
	/* enforce read-only access to this chrdev */
	if ((filp->f_mode & FMODE_READ) == 0)
		return -EINVAL;
	if (filp->f_mode & FMODE_WRITE)
		return -EINVAL;
	return 0;
}

static int qrng_op_release(struct inode *inode, struct file *filp)
{
	return 0;
}

#define CHRDEV_READ_SIZE 64
static ssize_t qrng_op_read(struct file *filp, char *buf,
								size_t count, loff_t *f_pos)
{
	int ret = 0;
	size_t read = 0;
	size_t total = count;
	char *pbuf = buf;

	unsigned char buffer[CHRDEV_READ_SIZE];

	if(filp->f_flags & O_NONBLOCK){
		PDBG("NONBLOCK STATUS is not supported\n");
		return 0;
	}

	if(buf == NULL || count < 0){
		PDBG("Invalid arguments\n");
		return 0;
	}

	while(total > 0) {
		read = total > CHRDEV_READ_SIZE ? CHRDEV_READ_SIZE : total;
		ret = qrng_get_data(buffer, read);

		if(ret){
			PERR("Fail to get_data on chrdev\n");
			memset(buffer, 0x00, CHRDEV_READ_SIZE);
			return -EINVAL;
		}

		if(copy_to_user(pbuf, buffer, read)){
			PERR("copy to user() failed\n");
			memset(buffer, 0x00, CHRDEV_READ_SIZE);
			return -EFAULT;
		}

		total -= read;
		pbuf  += read;
	}

	memset(buffer, 0x00, CHRDEV_READ_SIZE);
	return (count - total);
}

static ssize_t qrng_op_write(struct file *filp, const char __user *buf,
								size_t count, loff_t *f_pos)
{
	return 0;
}

static int qrng_diag(void);
static unsigned int get_elapse_ms(int bRestart);

static long qrng_op_ioctl(struct file *file,
							unsigned int cmd, unsigned long arg)
{
	PLINE;
	switch (cmd)
	{
	case QRNG_IOCTL_DIAG:
		return qrng_diag();
	}
	return -EINVAL;
}

#define	DIAG_READ_SIZE		(1024)
#define	DIAG_READ_CNT		(12)
#define	DIAG_BUFF_LEN		((DIAG_READ_CNT)*(DIAG_READ_SIZE))
#define	DIAG_TIMEOUT_MS		(2000)
#define	DIAG_DIFF_MAX		(20*2*DIAG_BUFF_LEN/100)	// 20 %

static int qrng_diag(void)
{
	int		ret, i, j, hist[4], max, min, diff;
	char	*kbuf, *p;
	unsigned int	elps_ms;

	PLINE;

	// memory allocation for kernel area
	kbuf = kvzalloc(DIAG_BUFF_LEN, GFP_KERNEL);
	if (kbuf == NULL)
	{
		PERR("kzalloc failed\n");
		return -ENOMEM;
	}

	// read rng data
	get_elapse_ms(1);
	for (i=0, p=kbuf; i<DIAG_READ_CNT; i++, p+=DIAG_READ_SIZE)
	{
		ret = qrng_get_data(p, DIAG_READ_SIZE);
		if (ret)
		{
			PERR("qrng_get_data error: %d\n", ret);
			kvfree(kbuf);
			return -EFAULT;
		}

		// check timeout
		elps_ms = get_elapse_ms(0);
		if (elps_ms > DIAG_TIMEOUT_MS)
		{
			PERR("diagnostic timeout: %d ms\n", elps_ms);
			kvfree(kbuf);
			return -ETIMEDOUT;
		}
	}

	// check uniformity
	hist[0] = hist[1] = hist[2] = hist[3] = 0;
	for (i=0; i<DIAG_BUFF_LEN; i++)
		for (j=6; j>=0; j-=2)
			hist[(kbuf[i]>>j)&3]++;
	PINFO("Histogram data : %d %d %d %d\n", hist[0], hist[1], hist[2], hist[3]);

	max = 0;
	min = DIAG_BUFF_LEN;
	for (i=0; i<4; i++)
	{
		if (hist[i] > max)	max = hist[i];
		if (hist[i] < min)	min = hist[i];
	}
	diff = max - min;
	PINFO("Diff : %d (%d - %d)\n", diff, max, min);
	if (diff > DIAG_DIFF_MAX)
	{
		PERR("diagnostic uniformity fail: %d (%d-%d)\n", diff, max, min);
		kvfree(kbuf);
		return -EINVAL;
	}

	elps_ms = get_elapse_ms(0);
	PINFO("Diagnosis elapsed time: %d ms\n", elps_ms);

	kvfree(kbuf);

	PINFO("QRNG : GOOD");
	return 0;
}

static unsigned int get_elapse_ms(int bRestart)
{
	static unsigned int	start_ms = 0;
	unsigned int	end_ms, elapse_ms;
	struct timespec	start, end;

	// get end time
	getnstimeofday(&end);
	end_ms = (end.tv_sec*1000) + (end.tv_nsec/1000000);

	// calculate elase time
	elapse_ms = end_ms - start_ms;

	// get start time for next check
	if (bRestart)
	{
		getnstimeofday(&start);
		start_ms = (start.tv_sec*1000) + (start.tv_nsec/1000000);
	}

	return elapse_ms;
}
#endif //CONFIG_HW_RANDOM_IDQ250C2_MISC_DEVICE

static struct file_operations qrng_fops = {
	.owner			= THIS_MODULE,
#ifdef CONFIG_HW_RANDOM_IDQ250C2_MISC_DEVICE
	.read			= qrng_op_read,
	.write			= qrng_op_write,
	.unlocked_ioctl	= qrng_op_ioctl,
	.open			= qrng_op_open,
	.release		= qrng_op_release,
#endif
};

static struct miscdevice qrng_misc = {
#ifdef CONFIG_HW_RANDOM_IDQ250C2_MISC_DEVICE
	.name		= "qrandom",
	.nodename	= "qrng-char",
#else
	.name		= "qrng_misc",
#endif
	.minor		= MISC_DYNAMIC_MINOR,
	.fops		= &qrng_fops,
};

// ---------------------------------------------------------------------------
// i2c protocol driver
// ---------------------------------------------------------------------------

#define PWDN_GPIO_NAME    "pwdn-gpio"

static int qrng_i2c_probe(struct i2c_client *client,
							const struct i2c_device_id *id)
{
	int		ret = -1;
	struct device	*dev = &client->dev;
	QRNG_DEV		*qrng;

	PLINE;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		PERR("Error : i2c_check_functionality");
		return -ENOTSUPP;
	}

	if ((qrng = kzalloc(sizeof(*qrng), GFP_KERNEL)) == NULL) {
		PERR("Error : kzalloc for QRND_DEV");
		return -ENOMEM;
	}

	qrng->dev_num = 0;
	scnprintf(qrng->dev_name, sizeof(qrng->dev_name), "qrng-%d", qrng->dev_num);
	PINFO("qrng->dev_name : %s", qrng->dev_name);

	qrng->dev = get_device(dev);
	dev_set_drvdata(dev, qrng);

	qrng->client = client;
	qrng->QRNG_READY_STATUS = 0x00; // Make sure

	mutex_init(&qrng->trans_mtx);


	qrng->GPIO_PWDN = of_get_named_gpio(dev->of_node, PWDN_GPIO_NAME, 0);
	if(qrng->GPIO_PWDN < 0 ){
		PERR("Failed to get %s, error: %d\n", PWDN_GPIO_NAME, qrng->GPIO_PWDN);
		return -ENODEV;
	}else{
		PINFO("Success to get %s :%d\n", PWDN_GPIO_NAME, qrng->GPIO_PWDN);
	}

	gQrng = qrng;

	// qrng initialization

	ret = gpio_request(gQrng->GPIO_PWDN, "QRNG_GPIO_PWDN");
	if (ret < 0) {
		PERR("Unable to request GPIO_PWDN : %d\n", ret);
		goto err;
	}

	gQrng->QRNG_READY_STATUS = QRNG_DEV_READY_FLAG;

	// set pwdn and ldo enable
	if(qrng_pwdn(0) < 0) {
		PERR("Failed to set qrng_pwdn : 0");
		return QRNG_INF_ERR;
	}

	gQrng->QRNG_PWDN_STATUS = 0;

	kbuf = kzalloc(CHUNK_SIZE, GFP_KERNEL);
	if (kbuf == NULL){
		PERR("kzalloc failed\n");
		ret = -ENOMEM;
		goto err;
	}

	// misc device
	qrng_misc.parent = qrng->dev;
	ret = misc_register (&qrng_misc);
	if (ret){
		PERR("misc_register error: %d\n", ret);
		goto err;
	}
	PINFO("misc_register: ok\n");

	memset(&qrng_rng, 0x00, sizeof (struct hwrng));
	qrng_rng.name		= "qrng";
	qrng_rng.read		= qrng_rng_read;
//	qrng_rng.quality 	= CONFIG_HW_RANDOM_IDQ250C2_QUALITY;

	// hwrng device
	ret = hwrng_register(&qrng_rng);
	if (ret){
		PERR("hwrng_register: %d\n", ret);
		goto err_hwrng;
	}

	PINFO("hwrng_register: ok\n");

	if(gQrng == NULL) {
		PERR("gQrng is null\n");
		goto err_hwrng;
	}

	return 0;

err_hwrng:
	misc_deregister(&qrng_misc);
err:
	PERR("Probe failed");
	gpio_free(qrng->GPIO_PWDN);
	gQrng = NULL;
	kzfree(qrng);
	return ret;
}

static int qrng_i2c_remove(struct i2c_client *client)
{
	struct device	*dev = &(client->dev);
	QRNG_DEV		*qrng = dev_get_drvdata(dev);

	PLINE;

#ifdef CONFIG_HW_RANDOM_IDQ250C2_TIMER
	kerneltimer_exit(&timer);
#endif

	hwrng_unregister(&qrng_rng);
	misc_deregister(&qrng_misc);

	gpio_free(qrng->GPIO_PWDN);

	gQrng = NULL;
	if(qrng)
		kzfree(qrng);

	if(kbuf)
		kzfree(kbuf);

	return 0;
}


static const struct i2c_device_id qrng_i2c_id[] = {
	{DEVICE_NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, qrng_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id qrng_of_table[] = {
 {.compatible = "qrng,qrng-i2c"},
 {},
};
MODULE_DEVICE_TABLE(of, qrng_of_table);
#endif

static struct i2c_driver qrng_i2c_driver = {
	.driver 	= {
		.name		= DEVICE_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = of_match_ptr(qrng_of_table),
	},
	.probe		= qrng_i2c_probe,
	.remove		= qrng_i2c_remove,
	.id_table	= qrng_i2c_id,
};

// ---------------------------------------------------------------------------
// module init / exit
// ---------------------------------------------------------------------------



static int __init qrng_mod_init(void)
{
	int	ret = 0;

	PLINE;

	ret = i2c_add_driver(&qrng_i2c_driver);

	if (ret){
		PERR("i2c_add_driver error: %d\n", ret);
		return ret;
	} else {
		PINFO("i2c_add_driver ok\n");
	}
	return ret;
}

static void __exit qrng_mod_exit(void)
{
	PLINE;
	i2c_del_driver(&qrng_i2c_driver);


}

module_init(qrng_mod_init);
module_exit(qrng_mod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hankoo Frank Lee <lehako@gmail.com>");
MODULE_DESCRIPTION("QRNG-IDQ250C2 Random Nubmer Generator (RNG) driver");

// ---------------------------------------------------------------------------
// end of this
// ---------------------------------------------------------------------------
