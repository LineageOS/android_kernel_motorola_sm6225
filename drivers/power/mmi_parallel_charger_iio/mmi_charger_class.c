/*
 * Copyright (c) 2018 Motorola Mobility, LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/module.h>
#include <linux/stat.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include "mmi_charger_class.h"

static struct class *mmi_charger_class;

int mmi_enable_charging(struct mmi_charger_device *chrg, bool en)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->enable)
		return chrg->ops->enable(chrg,en);

	return -ENOTSUPP;
}

int mmi_is_charging_enabled(struct mmi_charger_device *chrg, bool *en)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->is_enabled)
		return chrg->ops->is_enabled(chrg,en);

	return -ENOTSUPP;
}

int mmi_get_charing_current(struct mmi_charger_device *chrg, u32 *uA)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->get_charging_current)
		return chrg->ops->get_charging_current(chrg,uA);

	return -ENOTSUPP;
}

int mmi_set_charing_current(struct mmi_charger_device *chrg, u32 uA)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->set_charging_current)
		return chrg->ops->set_charging_current(chrg,uA);

	return -ENOTSUPP;
}

int mmi_get_input_current_settled(struct mmi_charger_device *chrg, u32 *uA)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->get_input_current_settled)
		return chrg->ops->get_input_current_settled(chrg,uA);

	return -ENOTSUPP;
}

int mmi_get_input_current(struct mmi_charger_device *chrg, u32 *uA)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->get_input_current)
		return chrg->ops->get_input_current(chrg,uA);

	return -ENOTSUPP;

}

int mmi_set_input_current(struct mmi_charger_device *chrg, u32 uA)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->set_input_current)
		return chrg->ops->set_input_current(chrg,uA);

	return -ENOTSUPP;
}

int mmi_update_charger_status(struct mmi_charger_device *chrg)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->update_charger_status)
		return chrg->ops->update_charger_status(chrg);

	return -ENOTSUPP;
}

int mmi_update_charger_error(struct mmi_charger_device *chrg)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->update_charger_error)
		return chrg->ops->update_charger_error(chrg);

	return -ENOTSUPP;
}

int mmi_clear_charger_error(struct mmi_charger_device *chrg)
{
	if(chrg != NULL && chrg->ops != NULL && chrg->ops->clear_charger_error)
		return chrg->ops->clear_charger_error(chrg);

	return -ENOTSUPP;
}

static void mmi_charger_device_release(struct device *dev)
{
	struct mmi_charger_device *charger_dev = to_mmi_charger_device(dev);

	kfree(charger_dev);
	return;
}

static int charger_match_device_by_name(struct device *dev,
	const void *data)
{
	const char *name = data;
	return strcmp(dev_name(dev), name) == 0;
}

struct mmi_charger_device *get_charger_by_name(const char *name)
{
	struct device *dev;

	if(!name)
		return (struct mmi_charger_device *)NULL;
	dev = class_find_device(mmi_charger_class, NULL, name,
						charger_match_device_by_name);

	return dev ? to_mmi_charger_device(dev) : NULL;
}

int is_charger_exist(const char *name)
{
	if (get_charger_by_name(name) == NULL)
		return 0;
	return 1;
}

struct mmi_charger_device *mmi_charger_device_register(const char *name,
		const char *psy_name,struct device *parent, void *devdata,
		const struct mmi_charger_ops *ops)
{
	struct mmi_charger_device *charger_dev;
	int rc;

	pr_info("mmi charger device register: name=%s\n",name);
	charger_dev = kzalloc(sizeof(struct mmi_charger_device),GFP_KERNEL);
	if (!charger_dev)
		return ERR_PTR(-ENOMEM);

	mutex_init(&charger_dev->ops_lock);
	charger_dev->dev.class = mmi_charger_class;
	charger_dev->dev.parent = parent;
	charger_dev->dev.release = mmi_charger_device_release;
	charger_dev->name = name;
	dev_set_name(&charger_dev->dev, "%s", name);
	dev_set_drvdata(&charger_dev->dev, devdata);

	rc = device_register(&charger_dev->dev);
	if (rc) {
		kfree(charger_dev);
		return ERR_PTR(rc);
	}
	charger_dev->chrg_psy = power_supply_get_by_name(psy_name);
	charger_dev->ops = ops;
	if (!charger_dev->chrg_psy)
		return ERR_PTR(-ENODEV);
	pr_info("mmi charger device register: name=%s, successfully\n",name);
	return charger_dev;
}

EXPORT_SYMBOL(mmi_charger_device_register);

void mmi_charger_device_unregister(struct mmi_charger_device* charger_dev)
{
	if (!charger_dev)
		return;

	mutex_lock(&charger_dev->ops_lock);
	charger_dev->ops = NULL;
	mutex_unlock(&charger_dev->ops_lock);
	device_unregister(&charger_dev->dev);
}

EXPORT_SYMBOL(mmi_charger_device_unregister);

void mmi_charger_class_exit(void)
{
	class_destroy(mmi_charger_class);
}

int mmi_charger_class_init(void)
{
	mmi_charger_class = class_create(THIS_MODULE, "mmi_charger");
	if (IS_ERR(mmi_charger_class)) {
		pr_err("Unable to create mmi charger class; error = %ld\n",
			PTR_ERR(mmi_charger_class));
		return PTR_ERR(mmi_charger_class);
	}
	pr_info("success to create mmi charger class \n");
	return 0;
}
