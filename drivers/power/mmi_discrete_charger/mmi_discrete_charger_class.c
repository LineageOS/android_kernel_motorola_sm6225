/* Copyright (c) 2020, 2021 The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <linux/mmi_discrete_charger_class.h>

static struct class *charger_class;

int charger_dev_is_charge_halted(struct charger_device *chg_dev, bool *en)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->is_charge_halted)
		return chg_dev->ops->is_charge_halted(chg_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_is_charge_halted);

int charger_dev_set_constant_voltage(struct charger_device *chg_dev, u32 uV)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->set_constant_voltage)
		return chg_dev->ops->set_constant_voltage(chg_dev, uV);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_constant_voltage);

int charger_dev_set_charging_current(struct charger_device *chg_dev, u32 uA)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->set_charging_current)
		return chg_dev->ops->set_charging_current(chg_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_charging_current);

int charger_dev_get_charging_current(struct charger_device *chg_dev, u32 *uA)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->get_charging_current)
		return chg_dev->ops->get_charging_current(chg_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_charging_current);

int charger_dev_enable_charging(struct charger_device *chg_dev, bool en)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->enable_charging)
		return chg_dev->ops->enable_charging(chg_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_charging);

int charger_dev_enable_hz(struct charger_device *chg_dev, bool en)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->enable_hz)
		return chg_dev->ops->enable_hz(chg_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_hz);

int charger_dev_enable_termination(struct charger_device *chg_dev, bool en)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->enable_termination)
		return chg_dev->ops->enable_termination(chg_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_termination);

int charger_dev_is_enabled_charging(struct charger_device *chg_dev, bool *en)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->is_enabled_charging)
		return chg_dev->ops->is_enabled_charging(chg_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_is_enabled_charging);

int charger_dev_dump_registers(struct charger_device *chg_dev, struct seq_file *m)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->dump_registers)
		return chg_dev->ops->dump_registers(chg_dev, m);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_dump_registers);

int charger_dev_rerun_aicl(struct charger_device *chg_dev)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->rerun_aicl)
		return chg_dev->ops->rerun_aicl(chg_dev);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_rerun_aicl);

int charger_dev_enable_otg(struct charger_device *chg_dev, bool en)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->enable_otg)
		return chg_dev->ops->enable_otg(chg_dev, en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_otg);

int charger_dev_set_boost_current_limit(struct charger_device *chg_dev, u32 uA)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->set_boost_current_limit)
		return chg_dev->ops->set_boost_current_limit(chg_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_boost_current_limit);

int charger_dev_enable_hw_jeita(struct charger_device *chg_dev, bool enable)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->enable_hw_jeita)
		return chg_dev->ops->enable_hw_jeita(chg_dev, enable);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_enable_hw_jeita);

int charger_dev_get_input_current(struct charger_device *chg_dev, u32 *uA)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->get_input_current)
		return chg_dev->ops->get_input_current(chg_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_input_current);

int charger_dev_set_input_current(struct charger_device *chg_dev, u32 uA)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->set_input_current)
		return chg_dev->ops->set_input_current(chg_dev, uA);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_input_current);

int charger_dev_set_input_voltage(struct charger_device *chg_dev, u32 uV)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->set_input_voltage_limit)
		return chg_dev->ops->set_input_voltage_limit(chg_dev, uV);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_input_voltage);

int charger_dev_is_usb_suspend(struct charger_device *chg_dev, bool *suspend_en)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->is_usb_suspend)
		return chg_dev->ops->is_usb_suspend(chg_dev, suspend_en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_is_usb_suspend);

int charger_dev_set_usb_suspend(struct charger_device *chg_dev, bool suspend_en)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->set_usb_suspend)
		return chg_dev->ops->set_usb_suspend(chg_dev, suspend_en);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_usb_suspend);

int charger_dev_get_pulse_cnt(struct charger_device *chg_dev, int *count)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->get_pulse_cnt)
		return chg_dev->ops->get_pulse_cnt(chg_dev, count);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_pulse_cnt);

int charger_dev_set_dp_dm(struct charger_device *chg_dev, int val)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->set_dp_dm)
		return chg_dev->ops->set_dp_dm(chg_dev, val);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_set_dp_dm);

int charger_dev_get_real_charger_type(struct charger_device *chg_dev, int *charger_type)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->get_real_charger_type)
		return chg_dev->ops->get_real_charger_type(chg_dev, charger_type);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_real_charger_type);

int charger_dev_get_qc3p_power(struct charger_device *chg_dev, int *qc3p_power)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->get_qc3p_power)
		return chg_dev->ops->get_qc3p_power(chg_dev, qc3p_power);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_get_qc3p_power);

int charger_dev_config_pd_active(struct charger_device *chg_dev, int val)
{
	if (chg_dev != NULL && chg_dev->ops != NULL &&
	    chg_dev->ops->config_pd_active)
		return chg_dev->ops->config_pd_active(chg_dev, val);

	return -ENOTSUPP;
}
EXPORT_SYMBOL(charger_dev_config_pd_active);

int charger_dev_notify(struct charger_device *chg_dev)
{
	return srcu_notifier_call_chain(
		&chg_dev->evt_nh, 0, &chg_dev->noti);
}
EXPORT_SYMBOL(charger_dev_notify);

int register_charger_device_notifier(struct charger_device *chg_dev,
				struct notifier_block *nb)
{
	int ret;

	ret = srcu_notifier_chain_register(&chg_dev->evt_nh, nb);
	return ret;
}
EXPORT_SYMBOL(register_charger_device_notifier);

int unregister_charger_device_notifier(struct charger_device *chg_dev,
				struct notifier_block *nb)
{
	return srcu_notifier_chain_unregister(&chg_dev->evt_nh, nb);
}
EXPORT_SYMBOL(unregister_charger_device_notifier);

static void charger_device_release(struct device *dev)
{
	struct charger_device *chg_dev = to_charger_device(dev);

	kfree(chg_dev);
}

/**
 * charger_device_register - create and register a new object of
 *   charger_device class.
 * @name: the name of the new object
 * @parent: a pointer to the parent device
 * @devdata: an optional pointer to be stored for private driver use.
 * The methods may retrieve it by using charger_get_data(charger_dev).
 * @ops: the charger operations structure.
 *
 * Creates and registers new charger device. Returns either an
 * ERR_PTR() or a pointer to the newly allocated device.
 */
struct charger_device *charger_device_register(const char *name,
		struct device *parent, void *devdata,
		const struct charger_ops *ops,
		const struct charger_properties *props)
{
	struct charger_device *chg_dev;
	static struct lock_class_key key;
	struct srcu_notifier_head *head;
	int rc;

	pr_debug("%s: name=%s\n", __func__, name);
	chg_dev = kzalloc(sizeof(*chg_dev), GFP_KERNEL);
	if (!chg_dev)
		return ERR_PTR(-ENOMEM);

	head = &chg_dev->evt_nh;
	srcu_init_notifier_head(head);
	/* Rename srcu's lock to avoid LockProve warning */
	lockdep_init_map(&(&head->srcu)->dep_map, name, &key, 0);
	mutex_init(&chg_dev->ops_lock);
	chg_dev->dev.class = charger_class;
	chg_dev->dev.parent = parent;
	chg_dev->dev.release = charger_device_release;
	dev_set_name(&chg_dev->dev, "%s", name);
	dev_set_drvdata(&chg_dev->dev, devdata);

	/* Copy properties */
	if (props) {
		memcpy(&chg_dev->props, props,
		       sizeof(struct charger_properties));
	}
	rc = device_register(&chg_dev->dev);
	if (rc) {
		kfree(chg_dev);
		return ERR_PTR(rc);
	}
	chg_dev->ops = ops;
	return chg_dev;
}
EXPORT_SYMBOL(charger_device_register);

/**
 * charger_device_unregister - unregisters a discrete charger device
 * object.
 * @charger_dev: the discrete charger device object to be unregistered
 * and freed.
 *
 * Unregisters a previously registered via charger_device_register object.
 */
void charger_device_unregister(struct charger_device *chg_dev)
{
	if (!chg_dev)
		return;

	mutex_lock(&chg_dev->ops_lock);
	chg_dev->ops = NULL;
	mutex_unlock(&chg_dev->ops_lock);
	device_unregister(&chg_dev->dev);
}
EXPORT_SYMBOL(charger_device_unregister);

static int charger_match_device_by_name(struct device *dev,
	const void *data)
{
	const char *name = data;

	return strcmp(dev_name(dev), name) == 0;
}

struct charger_device *get_charger_by_name(const char *name)
{
	struct device *dev;

	if (!name)
		return (struct charger_device *)NULL;
	dev = class_find_device(charger_class, NULL, name,
				charger_match_device_by_name);

	return dev ? to_charger_device(dev) : NULL;

}
EXPORT_SYMBOL(get_charger_by_name);

static ssize_t name_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct charger_device *chg_dev = to_charger_device(dev);

	return snprintf(buf, 20, "%s\n",
		       chg_dev->props.alias_name ?
		       chg_dev->props.alias_name : "anonymous");
}

static DEVICE_ATTR_RO(name);

static struct attribute *charger_class_attrs[] = {
	&dev_attr_name.attr,
	NULL,
};

static const struct attribute_group charger_group = {
	.attrs = charger_class_attrs,
};

static const struct attribute_group *charger_groups[] = {
	&charger_group,
	NULL,
};

static void __exit charger_class_exit(void)
{
	class_destroy(charger_class);
}

static int __init charger_class_init(void)
{
	charger_class = class_create(THIS_MODULE, "mmi_discrete_charger");
	if (IS_ERR(charger_class)) {
		pr_err("Unable to create mmi discrete charger class; errno = %ld\n",
			PTR_ERR(charger_class));
		return PTR_ERR(charger_class);
	}
	charger_class->dev_groups = charger_groups;
	pr_info("success to create mmi discrete charger class \n");
	return 0;
}

module_init(charger_class_init);
module_exit(charger_class_exit);

MODULE_DESCRIPTION("Mmi Discrete Charger Class Device");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");

