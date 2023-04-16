#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/mmi_relay.h>
#include <linux/slab.h>

typedef struct _relay_node {
    struct list_head list;
    union {
        struct atomic_notifier_head atomic_head;
        struct blocking_notifier_head blocking_head;
    };
    int id;
    notifier_t type;
    notifier_d dev;
}relay_node;

struct mmi_relay_dev {
    struct platform_device *pdev;
    struct mutex lock;
    relay_node nodes[NUM_TYPES][NUM_DEVS];
    unsigned notifier_nums;
};
static struct mmi_relay_dev* relay_dev;

/*
 * Informs the registered notifiers about an event
 * @type: notification type: atomic blocking
 * @dev: notification sender
 * @val: value passed to notifier function
 */

int relay_notifier_fire(notifier_t type, notifier_d dev, unsigned long val, void *v)
{
    int ret = 0;

    if(!relay_dev) {
        pr_err("Device is not initialized\n");
        return -EINVAL;
    }

    if(type >= NUM_TYPES || dev >= NUM_DEVS) {
        pr_err("Relay error type or dev\n");
        return -EPERM;
    }
    switch(type) {
        case ATOMIC:
        {
            struct atomic_notifier_head* h = &relay_dev->nodes[type][dev].atomic_head;
            ret = atomic_notifier_call_chain(h, val, v);
            break;
        }
        case BLOCKING:
        {
            struct blocking_notifier_head* h = &relay_dev->nodes[type][dev].blocking_head;
            ret  = blocking_notifier_call_chain(h, val, v);
            break;
        }
        default:
            BUG_ON(1);
    }
    return ret;
}
EXPORT_SYMBOL_GPL(relay_notifier_fire);

/**
 * Register an atomic or blocking notify to receive notifications from dev.
 * @dev: notification sender
 */
int relay_register_action(notifier_t type, notifier_d dev, struct notifier_block * nb)
{
    int ret = 0;

    if(!relay_dev) {
        pr_err("Device is not initialized\n");
        return -EINVAL;
    }

    if(type >= NUM_TYPES || dev >= NUM_DEVS) {
        pr_err("Relay error type or dev\n");
        return -EPERM;
    }

    mutex_lock(&relay_dev->lock);
    switch(type) {
        case ATOMIC:
        {
            struct atomic_notifier_head* h = &relay_dev->nodes[type][dev].atomic_head;
            ret = atomic_notifier_chain_register(h, nb);
            if (!ret)
                relay_notifier_fire(ATOMIC, MMI_RELAY, RELAY_NOTIFY_REGISTER, NULL);
            break;
        }
        case BLOCKING:
        {
            struct blocking_notifier_head* h = &relay_dev->nodes[type][dev].blocking_head;
            ret = blocking_notifier_chain_register(h, nb);
            if (!ret)
                relay_notifier_fire(BLOCKING, MMI_RELAY, RELAY_NOTIFY_REGISTER, NULL);

            break;
        }
        default:
            BUG_ON(1);
    }
    mutex_unlock(&relay_dev->lock);

    return ret;
}
EXPORT_SYMBOL_GPL(relay_register_action);

int relay_unregister_action(notifier_t type, notifier_d dev, struct notifier_block *nb)
{
    int ret = 0;

    if(!relay_dev) {
        pr_err("Device is not initialized\n");
        return -EINVAL;
    }

    if(type >= NUM_TYPES || dev >= NUM_DEVS) {
        pr_err("Relay error type or dev\n");
        return -EPERM;
    }

    mutex_lock(&relay_dev->lock);
    switch(type) {
        case ATOMIC:
        {
            struct atomic_notifier_head* h = &relay_dev->nodes[type][dev].atomic_head;
            ret = atomic_notifier_chain_unregister(h, nb);
            if (!ret)
                relay_notifier_fire(ATOMIC, MMI_RELAY, RELAY_NOTIFY_UNREGISTER, NULL);
            break;
        }
        case BLOCKING:
        {
            struct blocking_notifier_head* h = &relay_dev->nodes[type][dev].blocking_head;
            ret = blocking_notifier_chain_unregister(h, nb);
            if (!ret)
                relay_notifier_fire(BLOCKING, MMI_RELAY, RELAY_NOTIFY_UNREGISTER, NULL);
            break;
        }
        default:
            BUG_ON(1);
    }
    mutex_unlock(&relay_dev->lock);

    return ret;
}
EXPORT_SYMBOL_GPL(relay_unregister_action);

static int mmi_relay_probe(struct platform_device *pdev)
{
    notifier_t i;
    notifier_d j;

    relay_dev = kzalloc(sizeof(struct mmi_relay_dev), GFP_KERNEL);
    if (!relay_dev) {
        pr_err("Failed to allocate memory for relay_dev\n");
        return -ENOMEM;
    }

    relay_dev->pdev = pdev;
    relay_dev->notifier_nums = NUM_DEVS * NUM_TYPES;

    mutex_init(&relay_dev->lock);

    for(i = (notifier_t)0; i < NUM_TYPES; i++) {
        for(j = (notifier_d)0; j < NUM_DEVS; j++) {
            relay_dev->nodes[i][j].type = i;
            relay_dev->nodes[i][j].dev= j;
            relay_dev->nodes[i][j].id = i << 16 | j;
            INIT_LIST_HEAD(&relay_dev->nodes[i][j].list);

            if(i == ATOMIC) {
                ATOMIC_INIT_NOTIFIER_HEAD(&relay_dev->nodes[i][j].atomic_head);
            } else {
                BLOCKING_INIT_NOTIFIER_HEAD(&relay_dev->nodes[i][j].blocking_head);
            }
        }
    }
    return 0;
}

static int mmi_relay_remove(struct platform_device *pdev)
{
    kfree(relay_dev);
    return 0;
}

static const struct of_device_id mmi_relay_mt[] = {
	{.compatible = "mmi,sys-relay"},
	{},
};
MODULE_DEVICE_TABLE(of, mmi_relay_mt);

static struct platform_driver mmi_relay_driver = {
	.probe = mmi_relay_probe,
	.remove = mmi_relay_remove,
	.driver = {
		.name = "mmi_relay",
		.owner = THIS_MODULE,
		.of_match_table = mmi_relay_mt,
	},
};

static int __init mmi_relay_init(void)
{
	return platform_driver_register(&mmi_relay_driver);
}

static void __exit mmi_relay_exit(void)
{
	platform_driver_unregister(&mmi_relay_driver);
}

module_init(mmi_relay_init);
module_exit(mmi_relay_exit);

MODULE_ALIAS("platform:mmi_relay");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("Motorola Mobility Module Conmmunication Relay");
MODULE_LICENSE("GPL");
