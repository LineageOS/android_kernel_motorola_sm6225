#ifndef __JIIOV_NETLINK_H__
#define __JIIOV_NETLINK_H__

typedef enum {
    ANC_NETLINK_EVENT_TEST = 0,
    ANC_NETLINK_EVENT_IRQ,
    ANC_NETLINK_EVENT_SCR_OFF,
    ANC_NETLINK_EVENT_SCR_ON,
    ANC_NETLINK_EVENT_TOUCH_DOWN,
    ANC_NETLINK_EVENT_TOUCH_UP,
    ANC_NETLINK_EVENT_UI_READY,
    ANC_NETLINK_EVENT_EXIT,
    ANC_NETLINK_EVENT_INVALID,
    ANC_NETLINK_EVENT_MAX
} ANC_NETLINK_EVENT_TYPE;

int netlink_send_message_to_user(const char *p_buffer, size_t length);
int anc_netlink_init(void);
void anc_netlink_exit(void);

#endif