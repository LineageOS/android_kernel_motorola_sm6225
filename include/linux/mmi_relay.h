#ifndef __MMI_RELAY_H__
#define __MMI_RELAY_H__

#include <linux/notifier.h>

typedef enum notifier_type {
    ATOMIC,
    BLOCKING,
    NUM_TYPES,
}notifier_t;

typedef enum notifier_dev {
    LCD,
    FPS,
    MMI_RELAY,
    NUM_DEVS,
}notifier_d;

enum mmi_relay_event {
    RELAY_NOTIFY_REGISTER,
    RELAY_NOTIFY_UNREGISTER,
};

extern int relay_notifier_fire(notifier_t type, notifier_d dev, unsigned long val, void *v);
extern int relay_register_action(notifier_t type, notifier_d dev, struct notifier_block * nb);
extern int relay_unregister_action(notifier_t type, notifier_d dev, struct notifier_block * nb);

#endif
