#ifndef _LDC2114_CDEV_HDR_
#define _LDC2114_CDEV_HDR_

#include <linux/notifier.h>

#if defined(LDC2114_CHARDEV)
int ldc2114_cdev_init(void);
void ldc2114_cdev_remove(void);
int ldc2114_buffer(int d0, int d1, int d2, int d3);
int ldc2114_register_client(struct notifier_block *nb);
#else
static int ldc2114_cdev_init(void)
{
	return -ENODEV;
}
static void ldc2114_cdev_remove(void)
{
}
static int ldc2114_buffer(int d0, int d1, int d2, int d3)
{
	return 0;
}
static int ldc2114_register_client(struct notifier_block *nb)
{
	return -ENODEV;
}
#endif
#endif
