/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ili9882.h"
#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
#include <linux/mmi_device.h>
#endif
#ifdef CONFIG_DRM
struct drm_panel *ili_active_panel;
#ifdef ILI_FW_PANEL
static const char *active_panel_name = NULL;
#endif
#ifndef CONFIG_INPUT_TOUCHSCREEN_MMI
static int check_dt(struct device_node *np);
#endif
#endif

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
extern int ili_mmi_init(struct ilitek_ts_data *ts_data, bool enable);
#endif

struct touch_bus_info {
	struct spi_driver bus_driver;
	struct ilitek_hwif_info *hwif;
};

struct ilitek_ts_data *ilits;

#if SPI_DMA_TRANSFER_SPLIT
#define DMA_TRANSFER_MAX_CHUNK		64   // number of chunks to be transferred.
#define DMA_TRANSFER_MAX_LEN		4096 // length of a chunk.

int ili_spi_write_then_read_split(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	int status = -1, duplex_len = 0;
	int xfercnt = 0, xferlen = 0, xferloop = 0;
	int offset = 0;
	u8 cmd = 0;
	struct spi_message message;
	struct spi_transfer *xfer;

	xfer = kzalloc(DMA_TRANSFER_MAX_CHUNK * sizeof(struct spi_transfer), GFP_KERNEL);

	if (n_rx > SPI_RX_BUF_SIZE) {
		ILI_ERR("Rx length is greater than spi local buf, abort\n");
		status = -ENOMEM;
		goto out;
	}

	spi_message_init(&message);
	memset(ilits->spi_tx, 0x0, SPI_TX_BUF_SIZE);
	memset(ilits->spi_rx, 0x0, SPI_RX_BUF_SIZE);

	if ((n_tx > 0) && (n_rx > 0))
		cmd = SPI_READ;
	else
		cmd = SPI_WRITE;

	switch (cmd) {
	case SPI_WRITE:
		if (n_tx % DMA_TRANSFER_MAX_LEN)
			xferloop = (n_tx / DMA_TRANSFER_MAX_LEN) + 1;
		else
			xferloop = n_tx / DMA_TRANSFER_MAX_LEN;

		xferlen = n_tx;
		memcpy(ilits->spi_tx, (u8 *)txbuf, xferlen);

		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_LEN)
				xferlen = DMA_TRANSFER_MAX_LEN;

			xfer[xfercnt].len = xferlen;
			xfer[xfercnt].tx_buf = ilits->spi_tx + xfercnt * DMA_TRANSFER_MAX_LEN;
			spi_message_add_tail(&xfer[xfercnt], &message);
			xferlen = n_tx - (xfercnt+1) * DMA_TRANSFER_MAX_LEN;
		}
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		if (n_tx > DMA_TRANSFER_MAX_LEN) {
			ILI_ERR("Tx length must be lower than dma length (%d).\n", DMA_TRANSFER_MAX_LEN);
			status = -EINVAL;
			break;
		}

		if (!atomic_read(&ilits->ice_stat))
			offset = 2;

		memcpy(ilits->spi_tx, txbuf, n_tx);
		duplex_len = n_tx + n_rx + offset;

		if (duplex_len % DMA_TRANSFER_MAX_LEN)
			xferloop = (duplex_len / DMA_TRANSFER_MAX_LEN) + 1;
		else
			xferloop = duplex_len / DMA_TRANSFER_MAX_LEN;

		xferlen = duplex_len;
		for (xfercnt = 0; xfercnt < xferloop; xfercnt++) {
			if (xferlen > DMA_TRANSFER_MAX_LEN)
				xferlen = DMA_TRANSFER_MAX_LEN;

			xfer[xfercnt].len = xferlen;
			xfer[xfercnt].tx_buf = ilits->spi_tx;
			xfer[xfercnt].rx_buf = ilits->spi_rx + xfercnt * DMA_TRANSFER_MAX_LEN;
			spi_message_add_tail(&xfer[xfercnt], &message);
			xferlen = duplex_len - (xfercnt + 1) * DMA_TRANSFER_MAX_LEN;
		}

		status = spi_sync(spi, &message);
		if (status == 0) {
			if (ilits->spi_rx[1] != SPI_ACK && !atomic_read(&ilits->ice_stat)) {
				status = DO_SPI_RECOVER;
				ILI_ERR("Do spi recovery: rxbuf[1] = 0x%x, ice = %d\n", ilits->spi_rx[1], atomic_read(&ilits->ice_stat));
				break;
			}

			memcpy((u8 *)rxbuf, ilits->spi_rx + offset + 1, n_rx);
		} else {
			ILI_ERR("spi read fail, status = %d\n", status);
		}
		break;
	default:
		ILI_INFO("Unknown command 0x%x\n", cmd);
		break;
	}

out:
	ipio_kfree((void **)&xfer);
	return status;
}
#else
int ili_spi_write_then_read_direct(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	int status = -1, duplex_len = 0;
	int offset = 0;
	u8 cmd;
	struct spi_message message;
	struct spi_transfer xfer;

	if (n_rx > SPI_RX_BUF_SIZE) {
		ILI_ERR("Rx length is greater than spi local buf, abort\n");
		status = -ENOMEM;
		goto out;
	}

	spi_message_init(&message);
	memset(&xfer, 0, sizeof(xfer));

	if ((n_tx > 0) && (n_rx > 0))
		cmd = SPI_READ;
	else
		cmd = SPI_WRITE;

	switch (cmd) {
	case SPI_WRITE:
		xfer.len = n_tx;
		xfer.tx_buf = txbuf;
		spi_message_add_tail(&xfer, &message);
		status = spi_sync(spi, &message);
		break;
	case SPI_READ:
		if (!atomic_read(&ilits->ice_stat))
			offset = 2;

		duplex_len = n_tx + n_rx + offset;
		if ((duplex_len > SPI_TX_BUF_SIZE) ||
			(duplex_len > SPI_RX_BUF_SIZE)) {
			ILI_ERR("duplex_len is over than dma buf, abort\n");
			status = -ENOMEM;
			break;
		}

		memset(ilits->spi_tx, 0x0, SPI_TX_BUF_SIZE);
		memset(ilits->spi_rx, 0x0, SPI_RX_BUF_SIZE);

		xfer.len = duplex_len;
		memcpy(ilits->spi_tx, txbuf, n_tx);
		xfer.tx_buf = ilits->spi_tx;
		xfer.rx_buf = ilits->spi_rx;

		spi_message_add_tail(&xfer, &message);
		status = spi_sync(spi, &message);
		if (status == 0) {
			if (ilits->spi_rx[1] != SPI_ACK && !atomic_read(&ilits->ice_stat)) {
				status = DO_SPI_RECOVER;
				ILI_ERR("Do spi recovery: rxbuf[1] = 0x%x, ice = %d\n", ilits->spi_rx[1], atomic_read(&ilits->ice_stat));
				break;
			}

			memcpy((u8 *)rxbuf, ilits->spi_rx + offset + 1, n_rx);
		} else {
			ILI_ERR("spi read fail, status = %d\n", status);
		}
		break;
	default:
		ILI_INFO("Unknown command 0x%x\n", cmd);
		break;
	}

out:
	return status;
}
#endif

static int ili_spi_mp_pre_cmd(u8 cdc)
{
	u8 pre[5] = {0};

	if (!atomic_read(&ilits->mp_stat) || cdc != P5_X_SET_CDC_INIT ||
		ilits->chip->core_ver >= CORE_VER_1430)
		return 0;

	ILI_DBG("mp test with pre commands\n");

	pre[0] = SPI_WRITE;
	pre[1] = 0x0;// dummy byte
	pre[2] = 0x2;// Write len byte
	pre[3] = P5_X_READ_DATA_CTRL;
	pre[4] = P5_X_GET_CDC_DATA;
	if (ilits->spi_write_then_read(ilits->spi, pre, 5, NULL, 0) < 0) {
		ILI_ERR("Failed to write pre commands\n");
		return -1;
	}

	pre[0] = SPI_WRITE;
	pre[1] = 0x0;// dummy byte
	pre[2] = 0x1;// Write len byte
	pre[3] = P5_X_GET_CDC_DATA;
	if (ilits->spi_write_then_read(ilits->spi, pre, 4, NULL, 0) < 0) {
		ILI_ERR("Failed to write pre commands\n");
		return -1;
	}
	return 0;
}

static int ili_spi_pll_clk_wakeup(void)
{
	int index = 0;
	u8 wdata[32] = {0};
	u8 wakeup[9] = {0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3};
	u32 wlen = sizeof(wakeup);

	wdata[0] = SPI_WRITE;
	wdata[1] = wlen >> 8;
	wdata[2] = wlen & 0xff;
	index = 3;

	ipio_memcpy(&wdata[index], wakeup, wlen, wlen);
	wlen += index;

	ILI_INFO("Write dummy to wake up spi pll clk\n");
	if (ilits->spi_write_then_read(ilits->spi, wdata, wlen, NULL, 0) < 0) {
		ILI_INFO("spi slave write error\n");
		return -1;
	}
	mdelay(1);
	return 0;
}

static int ili_spi_wrapper(u8 *txbuf, u32 wlen, u8 *rxbuf, u32 rlen, bool spi_irq, bool i2c_irq)
{
	int ret = 0;
	int mode = 0, index = 0;
	u8 wdata[128] = {0};
	u8 checksum = 0;
	bool ice = atomic_read(&ilits->ice_stat);

	if (wlen > 0) {
		if (!txbuf) {
			ILI_ERR("txbuf is null\n");
			return -ENOMEM;
		}

		/* 3 bytes data consist of length and header */
		if ((wlen + 3) > sizeof(wdata)) {
			ILI_ERR("WARNING! wlen(%d) > wdata(%d), using wdata length to transfer\n", wlen, (int)sizeof(wdata));
			wlen = sizeof(wdata) - 3;
		}
	}

	if (rlen > 0) {
		if (!rxbuf) {
			ILI_ERR("rxbuf is null\n");
			return -ENOMEM;
		}
	}

	if (rlen > 0 && !wlen)
		mode = SPI_READ;
	else
		mode = SPI_WRITE;

	if (ilits->int_pulse)
		ilits->detect_int_stat = ili_ic_check_int_pulse;
	else
		ilits->detect_int_stat = ili_ic_check_int_level;

	if (spi_irq)
		atomic_set(&ilits->cmd_int_check, ENABLE);

	switch (mode) {
	case SPI_WRITE:
#if ( PLL_CLK_WAKEUP_TP_RESUME == ENABLE )
		if (ilits->pll_clk_wakeup == true) {
#else
		if ((ilits->pll_clk_wakeup == true) && (ilits->tp_suspend == true)) {
#endif
			ret = ili_spi_pll_clk_wakeup();
			if (ret < 0) {
				ILI_ERR("Wakeup pll clk error\n");
				break;
			}
		}
		if (ice) {
			wdata[0] = SPI_WRITE;
			index = 1;
		} else {
			wdata[0] = SPI_WRITE;
			wdata[1] = wlen >> 8;
			wdata[2] = wlen & 0xff;
			index = 3;
		}


		ipio_memcpy(&wdata[index], txbuf, wlen, wlen);
		wlen += index;

		/*
		* NOTE: If TP driver is doing MP test and commanding 0xF1 to FW, we add a checksum
		* to the last index and plus 1 with size.
		*/
		if (atomic_read(&ilits->mp_stat) && wdata[index] == P5_X_SET_CDC_INIT) {
			checksum = ili_calc_packet_checksum(&wdata[index], wlen - index);
			wdata[wlen] = checksum;
			wlen++;
			wdata[1] = (wlen - index) >> 8;
			wdata[2] = (wlen - index) & 0xff;
			ili_dump_data(wdata, 8, wlen, 0, "mp cdc cmd with checksum");
		}

		ret = ilits->spi_write_then_read(ilits->spi, wdata, wlen, txbuf, 0);

		if (!ice) {
			 ILI_INFO("send cmd delay 1ms\n");
			 mdelay(1);
			 }

		if (ret < 0) {
			ILI_INFO("spi-wrapper write error\n");
			break;
		}

		/* Won't break if it needs to read data following with writing. */
		if (!rlen)
			break;
	case SPI_READ:
		if (!ice && spi_irq) {
			/* Check INT triggered by FW when sending cmds. */
			if (ilits->detect_int_stat(false) < 0) {
				ILI_ERR("ERROR! Check INT timeout\n");
				ret = -ETIME;
				break;
			}
		}

		ret = ili_spi_mp_pre_cmd(wdata[3]);
		if (ret < 0)
			ILI_ERR("spi-wrapper mp pre cmd error\n");

		wdata[0] = SPI_READ;

		ret = ilits->spi_write_then_read(ilits->spi, wdata, 1, rxbuf, rlen);
		if (ret < 0)
			ILI_ERR("spi-wrapper read error\n");

		break;
	default:
		ILI_ERR("Unknown spi mode (%d)\n", mode);
		ret = -EINVAL;
		break;
	}

	if (spi_irq)
		atomic_set(&ilits->cmd_int_check, DISABLE);

	return ret;
}

int ili_core_spi_setup(int num)
{
	u32 freq[] = {
		TP_SPI_CLK_1M,
		TP_SPI_CLK_2M,
		TP_SPI_CLK_3M,
		TP_SPI_CLK_4M,
		TP_SPI_CLK_5M,
		TP_SPI_CLK_6M,
		TP_SPI_CLK_7M,
		TP_SPI_CLK_8M,
		TP_SPI_CLK_9M,
		TP_SPI_CLK_10M,
		TP_SPI_CLK_11M,
		TP_SPI_CLK_12M,
		TP_SPI_CLK_13M,
		TP_SPI_CLK_14M,
		TP_SPI_CLK_15M
	};

	if (num > sizeof(freq)) {
		ILI_ERR("Invaild clk freq, set default clk freq\n");
		num = 7;
	}

	ILI_INFO("spi clock = %d\n", freq[num]);

	ilits->spi->mode = SPI_MODE_0;
	ilits->spi->bits_per_word = 8;
	ilits->spi->max_speed_hz = freq[num];
	ilits->spi->chip_select = 0;

	if (spi_setup(ilits->spi) < 0) {
		ILI_ERR("Failed to setup spi device\n");
		return -ENODEV;
	}

	ILI_INFO("name = %s, bus_num = %d,cs = %d, mode = %d, speed = %d\n",
			ilits->spi->modalias,
			ilits->spi->master->bus_num,
			ilits->spi->chip_select,
			ilits->spi->mode,
			ilits->spi->max_speed_hz);
	return 0;
}

#ifdef ILI_FW_PANEL
static int ili_parse_tp_module()
{
   int tp_module = 0;
	if(active_panel_name) {
		if (strstr(active_panel_name, "txd")) {
			if (strstr(active_panel_name, "ili9882h")) {
				tp_module = MODEL_TXD_9882H;
			} else if (strstr(active_panel_name, "ili9882n")) {
				tp_module = MODEL_TXD_9882N;
			} else if (strstr(active_panel_name, "ili7806s")) {
				tp_module = MODEL_TXD_7806S;
			}
		} else if (strstr(active_panel_name, "tm")) { // || strstr(active_panel_name, "tianma")) {
			if (strstr(active_panel_name, "ili9882n")) {
				tp_module = MODEL_TM_9882N;
			} else if (strstr(active_panel_name, "ili9882h")) {
				tp_module = MODEL_TM_9882H;
			} else if (strstr(active_panel_name, "ili7807s")) {
				tp_module = MODEL_TM_7807S;
			}
		} else if (strstr(active_panel_name, "tianma") && strstr(active_panel_name, "ili9882n")) {
			tp_module = MODEL_TIANMA_9882N;
		} else if (strstr(active_panel_name, "csot")) {
			if (strstr(active_panel_name, "ili7807s")) {
				tp_module = MODEL_CSOT_7807S;
			}
		} else if (strstr(active_panel_name, "txd")) {
			if (strstr(active_panel_name, "ili7807s")) {
				tp_module = MODEL_TXD_7807S;
			}
		}
	}
	ILI_INFO("ili_parse_tp_module=%d\n", tp_module);
	return tp_module;
}
#endif //ILI_FW_PANEL

#ifndef CONFIG_INPUT_TOUCHSCREEN_MMI
#ifdef CONFIG_DRM
static int check_dt(struct device_node *np)
{
	int i;
	int count;
	int ret = -ENODEV;
	bool dts_using_dummy = false;
#ifdef ILI_FW_PANEL
	static int retry = 0;
#endif
	struct device_node *node;
	struct drm_panel *panel;

	count = of_count_phandle_with_args(np, "panel", NULL);
	if (count <= 0)
		return 0;

	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		ILI_INFO("node->name %s !\n", node->name);
		if(strstr(node->name, "dummy")) {
			dts_using_dummy = true;
		}
		of_node_put(node);
#ifdef ILI_FW_PANEL
		if (!IS_ERR(panel)) {
			ili_active_panel = panel;
			active_panel_name = node->name;
			ILI_INFO("%s: actived\n", active_panel_name);
			ret = 0;
		}
	}

		if(dts_using_dummy && ret) {
			ILI_INFO("Retry times %d.\n", retry);
			ret = -EPROBE_DEFER;
			if(retry++ > 5)
				ret = -ENODEV;
		}

		if(active_panel_name != NULL) {
			if(strstr(active_panel_name, "dummy")) {
				ILI_INFO("Using dummy panel! Return!\n");
				ret = -ENODEV;
			}
		}
#else
		if (!IS_ERR(panel)) {
			ili_active_panel = panel;
			pr_err("%s: ili9882h HLT actived\n", __func__);
			return MODEL_HLT;
		} else {
			node = of_parse_phandle(np, "panel2", i);
			panel = of_drm_find_panel(node);
			of_node_put(node);
			if (!IS_ERR(panel)) {
				ili_active_panel = panel;
				pr_err("%s: ili9882n TM actived\n", __func__);
				return MODEL_TM;
			}
		}
	}
#endif    //ILI_FW_PANEL

	if (node)
		pr_err("%s: %s not actived\n", __func__, node->name);
	return ret;
}
#endif
#endif

#ifdef CONFIG_DRM
static int parse_dt(struct device_node *np)
{
	int32_t ret = 0;

#ifdef ILI_SENSOR_EN
	if (of_property_read_bool(np, "ilitek,report_gesture_key")) {
		ILI_INFO("ilitek,report_gesture_key set");
		ilits->report_gesture_key = 1;
	} else {
		ilits->report_gesture_key = 0;
	}
#endif

#ifdef ILI_CONFIG_PANEL_GESTURE
	//parse gesture by panel config
	if (active_panel_name) {
		const char *panel_gesture;
		int panel_num = of_property_count_strings(np, "ilitek,gesture_panel");
		if (panel_num > 0) {
			int i;
			ILI_INFO("%s: get ilitek,gesture_panel count=%d", __func__, panel_num);
			for (i = 0; i < panel_num; i++) {
				ret = of_property_read_string_index(np, "ilitek,gesture_panel", i, &panel_gesture);
				if (ret < 0) {
					ILI_INFO("%s: cannot parse gesture_panel, ret=%d\n", __func__, ret);
					break;
				} else if (panel_gesture && strstr(active_panel_name, panel_gesture)) {
					ILI_INFO("%s: panel %s gesture enabled", __func__, panel_gesture);
					ilits->panel_gesture_enable = true;
					break;
				}
			}
		}
	}
#endif

  return ret;
}
#endif

#ifdef ILITEK_PEN_NOTIFIER
#define ENABLE_PASSIVE_PEN_MODE_CMD 0x01
#define DISABLE_PASSIVE_PEN_MODE_CMD 0x00
static int pen_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	ILI_INFO("Received event(%lu) for pen detection\n", event);

	mutex_lock(&ilits->touch_mutex);
	if (event == PEN_DETECTION_PULL)
		ili_ic_func_ctrl("passive_pen", ENABLE_PASSIVE_PEN_MODE_CMD);
	else if (event == PEN_DETECTION_INSERT)
		ili_ic_func_ctrl("passive_pen", DISABLE_PASSIVE_PEN_MODE_CMD);
	mutex_unlock(&ilits->touch_mutex);

    	return 0;
}
#endif

static int ilitek_spi_probe(struct spi_device *spi)
{
	struct touch_bus_info *info =
	container_of(to_spi_driver(spi->dev.driver),
		struct touch_bus_info, bus_driver);
	int tp_module = 0;
#ifndef CONFIG_INPUT_TOUCHSCREEN_MMI
#ifdef ILI_FW_PANEL
	int ret;
#endif
#endif
	ILI_INFO("ilitek spi probe\n");

	if (!spi) {
		ILI_ERR("spi device is NULL\n");
		return -ENODEV;
	}

#ifndef CONFIG_INPUT_TOUCHSCREEN_MMI
#ifdef CONFIG_DRM
{
	struct device_node *dp = spi->dev.of_node;

#ifdef ILI_FW_PANEL
	ret = check_dt(dp);
	if (ret) {
		ILI_INFO("panel error\n");
		return ret;
	}
#else
	if ((tp_module = check_dt(dp)) < 0) {
		ILI_ERR("%s: %s not actived\n", __func__, dp->name);
		return -ENODEV;
	}
#endif
}
#endif
#endif
#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
	if (spi->dev.of_node && !mmi_device_is_available(spi->dev.of_node)) {
		ILI_ERR("mmi: device not supported\n");
		return -ENODEV;
	}
#endif
	ilits = devm_kzalloc(&spi->dev, sizeof(struct ilitek_ts_data), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ilits)) {
		ILI_ERR("Failed to allocate ts memory, %ld\n", PTR_ERR(ilits));
		return -ENOMEM;
	}

	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		ILI_ERR("Full duplex not supported by master\n");
		return -EIO;
	}

	ilits->update_buf = kzalloc(MAX_HEX_FILE_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(ilits->update_buf)) {
		ILI_ERR("fw kzalloc error\n");
		return -ENOMEM;
	}

	/* Used for receiving touch data only, do not mix up with others. */
	ilits->tr_buf = kzalloc(TR_BUF_SIZE, GFP_ATOMIC);
	if (ERR_ALLOC_MEM(ilits->tr_buf)) {
		ILI_ERR("failed to allocate touch report buffer\n");
		return -ENOMEM;
	}

	ilits->spi_tx = kzalloc(SPI_TX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(ilits->spi_tx)) {
		ILI_ERR("Failed to allocate spi tx buffer\n");
		return -ENOMEM;
	}

	ilits->spi_rx = kzalloc(SPI_RX_BUF_SIZE, GFP_KERNEL | GFP_DMA);
	if (ERR_ALLOC_MEM(ilits->spi_rx)) {
		ILI_ERR("Failed to allocate spi rx buffer\n");
		return -ENOMEM;
	}

	ilits->gcoord = kzalloc(sizeof(struct gesture_coordinate), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ilits->gcoord)) {
		ILI_ERR("Failed to allocate gresture coordinate buffer\n");
		return -ENOMEM;
	}

//---parse tp module---
#ifdef ILI_FW_PANEL
	tp_module = ili_parse_tp_module();
#endif

	ilits->i2c = NULL;
	ilits->spi = spi;
	ilits->dev = &spi->dev;
	ilits->hwif = info->hwif;
	ilits->phys = "SPI";
	ilits->wrapper = ili_spi_wrapper;
	ilits->detect_int_stat = ili_ic_check_int_pulse;
	ilits->int_pulse = true;
	ilits->mp_retry = false;
	ilits->tp_module = tp_module;

#if SPI_DMA_TRANSFER_SPLIT
	ilits->spi_write_then_read = ili_spi_write_then_read_split;
#else
	ilits->spi_write_then_read = ili_spi_write_then_read_direct;
#endif

	ilits->actual_tp_mode = P5_X_FW_AP_MODE;
	ilits->tp_data_format = DATA_FORMAT_DEMO;
	ilits->tp_data_len = P5_X_DEMO_MODE_PACKET_LEN;

	if (TDDI_RST_BIND)
		ilits->reset = TP_IC_WHOLE_RST;
	else
		ilits->reset = TP_HW_RST_ONLY;

	ilits->rst_edge_delay = 10;
	ilits->fw_open = REQUEST_FIRMWARE;
	ilits->fw_upgrade_mode = UPGRADE_IRAM;
	ilits->mp_move_code = ili_move_mp_code_iram;
	ilits->gesture_move_code = ili_move_gesture_code_iram;
	ilits->esd_recover = ili_wq_esd_spi_check;
	ilits->ges_recover = ili_touch_esd_gesture_iram;
	ilits->gesture_mode = DATA_FORMAT_GESTURE_INFO;
	ilits->gesture_demo_ctrl = DISABLE;
	ilits->wtd_ctrl = OFF;
	ilits->report = ENABLE;
	ilits->netlink = DISABLE;
	ilits->dnp = DISABLE;
	ilits->irq_tirgger_type = IRQF_TRIGGER_FALLING;
	ilits->info_from_hex = ENABLE;
	ilits->wait_int_timeout = AP_INT_TIMEOUT;

	//---parse dts---
#ifdef CONFIG_DRM
	parse_dt(spi->dev.of_node);
#endif

#if ENABLE_GESTURE
#ifdef ILI_CONFIG_PANEL_GESTURE
  if (ilits->panel_gesture_enable)
#elif defined(ILI_CONFIG_GESTURE)
	if (MODEL_HLT == tp_module)
#endif
	{
	ilits->gesture = ENABLE;
	ilits->ges_sym.double_tap = DOUBLE_TAP;
	ilits->ges_sym.alphabet_line_2_top = ALPHABET_LINE_2_TOP;
	ilits->ges_sym.alphabet_line_2_bottom = ALPHABET_LINE_2_BOTTOM;
	ilits->ges_sym.alphabet_line_2_left = ALPHABET_LINE_2_LEFT;
	ilits->ges_sym.alphabet_line_2_right = ALPHABET_LINE_2_RIGHT;
	ilits->ges_sym.alphabet_m = ALPHABET_M;
	ilits->ges_sym.alphabet_w = ALPHABET_W;
	ilits->ges_sym.alphabet_c = ALPHABET_C;
	ilits->ges_sym.alphabet_E = ALPHABET_E;
	ilits->ges_sym.alphabet_V = ALPHABET_V;
	ilits->ges_sym.alphabet_O = ALPHABET_O;
	ilits->ges_sym.alphabet_S = ALPHABET_S;
	ilits->ges_sym.alphabet_Z = ALPHABET_Z;
	ilits->ges_sym.alphabet_V_down = ALPHABET_V_DOWN;
	ilits->ges_sym.alphabet_V_left = ALPHABET_V_LEFT;
	ilits->ges_sym.alphabet_V_right = ALPHABET_V_RIGHT;
	ilits->ges_sym.alphabet_two_line_2_bottom = ALPHABET_TWO_LINE_2_BOTTOM;
	ilits->ges_sym.alphabet_F = ALPHABET_F;
	ilits->ges_sym.alphabet_AT = ALPHABET_AT;

	ILI_INFO("ilitek: gesture eanble:%d\n", ilits->gesture);
	}
#endif

	if (ili_core_spi_setup(SPI_CLK) < 0)
		return -EINVAL;

#ifdef ILITEK_PEN_NOTIFIER
	ilits->pen_notif.notifier_call = pen_notifier_callback;
	pen_detection_register_client(&ilits->pen_notif);
#endif

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
	ili_mmi_init(ilits, true);
#endif

	return info->hwif->plat_probe();
}

static int ilitek_spi_remove(struct spi_device *spi)
{
	ILI_INFO();
	return 0;
}

static struct spi_device_id tp_spi_id[] = {
	{TDDI_DEV_ID, 0},
	{},
};

int ili_interface_dev_init(struct ilitek_hwif_info *hwif)
{
	struct touch_bus_info *info;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		ILI_ERR("faied to allocate spi_driver\n");
		return -ENOMEM;
	}

	if (hwif->bus_type != BUS_SPI) {
		ILI_ERR("Not SPI dev\n");
		ipio_kfree((void **)&info);
		return -EINVAL;
	}

	hwif->info = info;

	info->bus_driver.driver.name = hwif->name;
	info->bus_driver.driver.owner = hwif->owner;
	info->bus_driver.driver.of_match_table = hwif->of_match_table;
	info->bus_driver.driver.pm = hwif->pm;

	info->bus_driver.probe = ilitek_spi_probe;
	info->bus_driver.remove = ilitek_spi_remove;
	info->bus_driver.id_table = tp_spi_id;

	info->hwif = hwif;
	return spi_register_driver(&info->bus_driver);
}

void ili_interface_dev_exit(struct ilitek_ts_data *ts)
{
	struct touch_bus_info *info = (struct touch_bus_info *)ilits->hwif->info;

	ILI_INFO("remove spi dev\n");
	kfree(ilits->update_buf);
	kfree(ilits->spi_tx);
	kfree(ilits->spi_rx);
	spi_unregister_driver(&info->bus_driver);
	ipio_kfree((void **)&info);
}
