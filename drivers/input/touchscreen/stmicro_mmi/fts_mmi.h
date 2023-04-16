#ifndef _FTS_MMI_H_
#define _FTS_MMI_H_

#include <linux/gpio.h>
#include <linux/touchscreen_mmi.h>
#include <linux/mmi_wake_lock.h>

struct fts_ts_info;

#ifdef CONFIG_INPUT_TOUCHSCREEN_MMI
int fts_mmi_init(struct fts_ts_info *info, bool enable);
#else
static int inline fts_mmi_init(struct fts_ts_info *info, bool enable)
{
	return -ENOSYS;
}
#endif
#endif
