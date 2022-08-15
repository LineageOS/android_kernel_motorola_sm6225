// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt)	"BM_ULOG: %s: " fmt, __func__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/ipc_logging.h>
#include <linux/rpmsg.h>
#include <linux/soc/qcom/pmic_glink.h>
#include <linux/power/bm_adsp_ulog.h>

/* owner/type/opcodes for battery charger */
#define MSG_OWNER_BC                    32778
#define MSG_TYPE_REQ_RESP		1

#define BM_ULOG_GET			0x18
#define BM_ULOG_PROP_SET		0x19
#define BM_INIT_ULOG_GET		0x23

/* Generic definitions */
#define BM_ULOG_WAIT_TIME_MS		5000
#define MAX_ULOG_READ_BUFFER_SIZE	8192
#define BM_ULOG_PAGES			(50)

#define bm_info(bmdev, fmt, ...)		\
	do {					\
		pr_info(fmt, ##__VA_ARGS__);	\
		ipc_log_string(bmdev->ipc_log, fmt, ##__VA_ARGS__); \
	} while (0)

#define bm_dbg(bmdev, fmt, ...)			\
	do {					\
		if (*bmdev->debug_enabled)	\
			pr_info(fmt, ##__VA_ARGS__);	\
		else				\
			pr_debug(fmt, ##__VA_ARGS__);	\
		ipc_log_string(bmdev->ipc_log, fmt, ##__VA_ARGS__); \
	} while (0)

static bool debug_enabled;
module_param(debug_enabled, bool, 0600);
MODULE_PARM_DESC(debug_enabled, "Enable ulog to Kernel debug log");

struct bm_ulog_prop_req {
	struct pmic_glink_hdr	hdr;
	u64			categories;
	u32			level;
};

struct bm_ulog_req {
	struct pmic_glink_hdr	hdr;
	u32			max_logsize;
};

struct bm_ulog_resp {
	struct pmic_glink_hdr	hdr;
	char read_buffer[MAX_ULOG_READ_BUFFER_SIZE];
};

struct bm_ulog_dev {
	struct device			*dev;
	struct pmic_glink_client	*client;
	struct mutex			lock;
	struct completion		ack;
	u32				level;
	u64				categories;
	struct dentry			*debugfs_dir;
	bool				*debug_enabled;
	void				*ipc_log;
	char				ulog_buffer[MAX_ULOG_READ_BUFFER_SIZE];
};

static struct bm_ulog_dev *g_bmdev = NULL;

static int bm_ulog_write(struct bm_ulog_dev *bmdev, void *data, size_t len)
{
	int rc;

	mutex_lock(&bmdev->lock);
	reinit_completion(&bmdev->ack);
	rc = pmic_glink_write(bmdev->client, data, len);
	if (!rc) {
		rc = wait_for_completion_timeout(&bmdev->ack,
				msecs_to_jiffies(BM_ULOG_WAIT_TIME_MS));
		if (!rc) {
			pr_err("Error, timed out sending message\n");
			mutex_unlock(&bmdev->lock);
			return -ETIMEDOUT;
		}
		rc = 0;
	}
	mutex_unlock(&bmdev->lock);

	return rc;
}

static int bm_ulog_callback(void *priv, void *data, size_t len)
{
	struct pmic_glink_hdr *hdr = data;
	struct bm_ulog_dev *bmdev = priv;

	pr_debug("owner: %u type: %u opcode: %#x len: %zu\n", hdr->owner,
		hdr->type, hdr->opcode, len);

	switch (hdr->opcode) {
	case BM_ULOG_PROP_SET:
		complete(&bmdev->ack);
		break;
	case BM_ULOG_GET:
	case BM_INIT_ULOG_GET:
		if (len != sizeof(struct bm_ulog_resp)) {
			pr_err("Incorrect len %zu for bm ulog resp\n", len);
			break;
		}
		memcpy(bmdev->ulog_buffer,
				((struct bm_ulog_resp *)data)->read_buffer,
				MAX_ULOG_READ_BUFFER_SIZE);
		complete(&bmdev->ack);
		break;
	default:
		pr_err("Unknown opcode %u\n", hdr->opcode);
		break;
	}

	return 0;
}

static int bm_ulog_set_mask(struct bm_ulog_dev *bmdev,
		     enum bm_ulog_category_bitmap categories,
		     enum bm_ulog_level_type level)
{
	int rc;
	struct bm_ulog_prop_req prop_req = { { 0 } };

	if (level > BM_LOG_LEVEL_ALL_LOGS)
		level = BM_LOG_LEVEL_ALL_LOGS;

	prop_req.level = level;
	prop_req.categories = categories;
	prop_req.hdr.owner = MSG_OWNER_BC;
	prop_req.hdr.type = MSG_TYPE_REQ_RESP;
	prop_req.hdr.opcode = BM_ULOG_PROP_SET;

	rc = bm_ulog_write(bmdev, &prop_req, sizeof(prop_req));
	if (rc) {
		pr_err("BM ulog set mask failed\n");
		return rc;
	}

	bmdev->level = level;
	bmdev->categories = categories;
	pr_debug("BM ulog categories: 0x%llx, level: %d\n",
					(u64)categories, level);

	return 0;
}

static int bm_ulog_request_log(struct bm_ulog_dev *bmdev, u32 size)
{
	int rc;
	u32 max_logsize;
	struct bm_ulog_req ulog_req = { { 0 } };

	if (size > 0 && size < MAX_ULOG_READ_BUFFER_SIZE)
		max_logsize = size;
	else
		max_logsize = MAX_ULOG_READ_BUFFER_SIZE;

	ulog_req.hdr.owner = MSG_OWNER_BC;
	ulog_req.hdr.type = MSG_TYPE_REQ_RESP;
	ulog_req.hdr.opcode = BM_ULOG_GET;
	ulog_req.max_logsize = max_logsize;

	rc = bm_ulog_write(bmdev, &ulog_req, sizeof(ulog_req));

	return rc;
}

static int bm_ulog_request_init_log(struct bm_ulog_dev *bmdev, u32 size)
{
	int rc;
	u32 max_logsize;
	struct bm_ulog_req ulog_req = { { 0 } };

	if (size > 0 && size < MAX_ULOG_READ_BUFFER_SIZE)
		max_logsize = size;
	else
		max_logsize = MAX_ULOG_READ_BUFFER_SIZE;

	ulog_req.hdr.owner = MSG_OWNER_BC;
	ulog_req.hdr.type = MSG_TYPE_REQ_RESP;
	ulog_req.hdr.opcode = BM_INIT_ULOG_GET;
	ulog_req.max_logsize = max_logsize;

	rc = bm_ulog_write(bmdev, &ulog_req, sizeof(ulog_req));

	return rc;
}

int bm_ulog_get_log(char *buf, u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (*bmdev->debug_enabled == false) {
		pr_debug("BM ulog has not enabled yet\n");
		return -ENOTSUPP;
	}

	if (!buf) {
		pr_err("BM ulog invalid buf=%p\n", buf);
		return -EINVAL;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_request_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	memcpy(buf, bmdev->ulog_buffer, size);

	return 0;
}
EXPORT_SYMBOL(bm_ulog_get_log);

int bm_ulog_get_mask_log(enum bm_ulog_category_bitmap categories,
		    enum bm_ulog_level_type level,
		    char *buf, u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (*bmdev->debug_enabled == false) {
		pr_debug("BM ulog has not enabled yet\n");
		return -ENOTSUPP;
	}

	if (!buf) {
		pr_err("BM ulog invalid buf=%p\n", buf);
		return -EINVAL;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_set_mask(bmdev, categories, level);
	if (rc) {
		pr_err("BM ulog failed to set mask, rc=%d\n", rc);
		return rc;
	}

	rc = bm_ulog_request_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	memcpy(buf, bmdev->ulog_buffer, size);

	return 0;
}
EXPORT_SYMBOL(bm_ulog_get_mask_log);

static void bm_ulog_print_buffer(struct bm_ulog_dev *bmdev, u32 size)
{
	int i;
	int header = 0;

	for (i = 0; i < size; i++) {
		if (bmdev->ulog_buffer[i] == '\x0a') {
			bmdev->ulog_buffer[i] = '\0';
			bm_dbg(bmdev, "%s\n", &bmdev->ulog_buffer[header]);
			header = i + 1;
		} else if (bmdev->ulog_buffer[i] == '\0') {
			if (header < i) {
				bm_dbg(bmdev, "%s\n", &bmdev->ulog_buffer[header]);
			}
			break;
		}
	}
}

static int bm_ulog_print_init_log(u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_request_init_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	bm_ulog_print_buffer(bmdev, size);

	return 0;
}

int bm_ulog_print_log(u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (*bmdev->debug_enabled == false) {
		pr_debug("BM ulog has not enabled yet\n");
		return -ENOTSUPP;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_request_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	bm_ulog_print_buffer(bmdev, size);

	return 0;
}
EXPORT_SYMBOL(bm_ulog_print_log);

int bm_ulog_print_mask_log(enum bm_ulog_category_bitmap categories,
		      enum bm_ulog_level_type level, u32 size)
{
	int rc;
	struct bm_ulog_dev *bmdev = g_bmdev;

	if (!bmdev) {
		pr_err("BM ulog has not initialized yet\n");
		return -ENODEV;
	}

	if (*bmdev->debug_enabled == false) {
		pr_debug("BM ulog has not enabled yet\n");
		return -ENOTSUPP;
	}

	if (!size || size > MAX_ULOG_READ_BUFFER_SIZE) {
		pr_err("BM ulog invalid size=%d\n", size);
		return -EINVAL;
	}

	rc = bm_ulog_set_mask(bmdev, categories, level);
	if (rc) {
		pr_err("BM ulog failed to set mask, rc=%d\n", rc);
		return rc;
	}

	rc = bm_ulog_request_log(bmdev, size);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}

	bm_ulog_print_buffer(bmdev, size);

	return 0;
}
EXPORT_SYMBOL(bm_ulog_print_mask_log);

#ifdef CONFIG_DEBUG_FS
static int bm_ulog_dump_show(struct seq_file *s, void *unused)
{
	int rc;
	struct bm_ulog_dev *bmdev = s->private;

	rc = bm_ulog_set_mask(bmdev, bmdev->categories, bmdev->level);
	if (rc) {
		pr_err("BM ulog failed to set mask, rc=%d\n", rc);
		return rc;
	}

	rc = bm_ulog_request_log(bmdev, MAX_ULOG_READ_BUFFER_SIZE);
	if (rc) {
		pr_err("BM ulog failed to request log, rc=%d\n", rc);
		return rc;
	}
	seq_puts(s, bmdev->ulog_buffer);

	return 0;
}

static int bm_ulog_open(struct inode *inode, struct file *file)
{
	return single_open(file, bm_ulog_dump_show, inode->i_private);
}

static const struct file_operations bm_ulog_fops = {
	.open			= bm_ulog_open,
	.read			= seq_read,
	.llseek			= seq_lseek,
	.release		= single_release,
};

static void bm_ulog_add_debugfs(struct bm_ulog_dev *bmdev)
{
	int rc;
	struct dentry *dir, *file;

	dir = debugfs_create_dir("bm_ulog", NULL);
	if (IS_ERR(dir)) {
		rc = PTR_ERR(dir);
		pr_err("Failed to create bm ulog debugfs directory, rc=%d\n",
			rc);
		return;
	}

	file = debugfs_create_file("dump", 0444, dir, bmdev, &bm_ulog_fops);
	if (IS_ERR(file)) {
		rc = PTR_ERR(file);
		pr_err("Failed to create ulog dump debugfs file, rc=%d\n",
			rc);
		debugfs_remove_recursive(dir);
		return;
	}

	debugfs_create_x64("categories", 0664, dir, &bmdev->categories);
	debugfs_create_x32("level", 0664, dir, &bmdev->level);

	bmdev->debugfs_dir = dir;
}
#else
static void bm_ulog_add_debugfs(struct bm_ulog_dev *bmdev) { }
#endif

static int bm_ulog_probe(struct platform_device *pdev)
{
	int rc;
	struct bm_ulog_dev *bmdev;
	struct pmic_glink_client_data client_data = { };
	struct device_node *node = pdev->dev.of_node;
	bool init_log_enabled, init_debug_enabled;

	bmdev = devm_kzalloc(&pdev->dev, sizeof(*bmdev), GFP_KERNEL);
	if (!bmdev)
		return -ENOMEM;

	rc = of_property_read_u64(node, "categories", &bmdev->categories);
	if (rc)
		bmdev->categories = BM_ALL;
	rc = of_property_read_u32(node, "level", &bmdev->level);
	if (rc)
		bmdev->level = BM_LOG_LEVEL_INFO;

	init_log_enabled = of_property_read_bool(node, "init-log-enabled");

	bmdev->dev = &pdev->dev;
	client_data.id = MSG_OWNER_BC;
	client_data.name = "battery_manager_adsp_ulog";
	client_data.msg_cb = bm_ulog_callback;
	client_data.priv = bmdev;

	bmdev->client = pmic_glink_register_client(bmdev->dev, &client_data);
	if (IS_ERR(bmdev->client)) {
		rc = PTR_ERR(bmdev->client);
		if (rc != -EPROBE_DEFER)
			dev_err(bmdev->dev, "Error in registering with pmic_glink %d\n",
				rc);
		return rc;
	}

	mutex_init(&bmdev->lock);
	init_completion(&bmdev->ack);
	platform_set_drvdata(pdev, bmdev);
	bmdev->debug_enabled = &debug_enabled;
	g_bmdev = bmdev;
	bmdev->ipc_log = ipc_log_context_create(BM_ULOG_PAGES, "bm_ulog", 0);
	if (!bmdev->ipc_log)
		dev_err(bmdev->dev, "Failed to create ipc log\n");

	if (init_log_enabled) {
		init_debug_enabled = debug_enabled;
		debug_enabled = init_log_enabled;
		bm_ulog_print_log(MAX_ULOG_READ_BUFFER_SIZE);
		bm_ulog_print_init_log(MAX_ULOG_READ_BUFFER_SIZE);
		bm_ulog_print_mask_log(bmdev->categories, bmdev->level,
					MAX_ULOG_READ_BUFFER_SIZE);
		debug_enabled = init_debug_enabled;
	}
	bm_ulog_add_debugfs(bmdev);

	bm_info(bmdev, "BM adsp ulog driver initialized successfully\n");

	return 0;
}

static int bm_ulog_remove(struct platform_device *pdev)
{
	struct bm_ulog_dev *bmdev = platform_get_drvdata(pdev);
	int rc;

	ipc_log_context_destroy(bmdev->ipc_log);
	debugfs_remove_recursive(bmdev->debugfs_dir);
	rc = pmic_glink_unregister_client(bmdev->client);
	if (rc < 0) {
		pr_err("Error unregistering from pmic_glink, rc=%d\n", rc);
		return rc;
	}
	g_bmdev = NULL;

	return 0;
}

static const struct of_device_id bm_ulog_match_table[] = {
	{ .compatible = "qcom,bm-adsp-ulog" },
	{},
};

static struct platform_driver bm_ulog_driver = {
	.driver	= {
		.name = "bm_adsp_ulog",
		.of_match_table = bm_ulog_match_table,
	},
	.probe	= bm_ulog_probe,
	.remove	= bm_ulog_remove,
};
module_platform_driver(bm_ulog_driver);

MODULE_DESCRIPTION("QTI Glink battery manager adsp ulog driver");
MODULE_LICENSE("GPL v2");
