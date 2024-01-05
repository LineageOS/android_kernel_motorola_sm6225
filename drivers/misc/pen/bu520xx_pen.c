#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/pen_detection_notify.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#else
#include <linux/pm_wakeup.h>
#include <linux/mmi_wake_lock.h>
#endif

#define BU520XX_DBG_ENABLE
#ifdef BU520XX_DBG_ENABLE	//BU52055 for sofiap
#define pen_dbg(fmt, args...)  pr_info("[DBG] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#else
#define pen_dbg(fmt, args...)
#endif // BU520XX_DBG_ENABLE
#define pen_dbg_func_in()     pen_dbg("[BU520XX-DBG-F.IN] %s", __func__)
#define pen_dbg_func_out()    pen_dbg("[BU520XX-DBG-F.OUT] %s", __func__)
#define pen_dbg_line()        pen_dbg("[LINE] %d(%s)", __LINE__, __func__)

#define pen_err(fmt, args...)  printk("[err] %s(L%04d) : " fmt "\n", __func__, __LINE__, ##args)
#define pen_info(fmt, args...)  printk(fmt, ##args)

#define DRIVER_NAME "pen_detect"
static struct workqueue_struct *hall_sensor_wq;
static struct workqueue_struct *hall_sensor_do_wq;
static struct kobject *hall_sensor_kobj;

static int bu520xx_pen_probe(struct platform_device *pdev);
static int bu520xx_pen_remove(struct platform_device *pdev);

static struct input_device_id mID[] = {
	{ .driver_info = 1 },		//scan all device to match hall sensor
	{ },
};

static struct hall_sensor_str {
	int irq;
	int status;
	int gpio;
	int enable;
	spinlock_t mHallSensorLock;
	#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock wake_lock;
	#else
	struct wakeup_source *wake_lock;
	#endif
	struct input_dev *pen_indev;
	struct input_handler pen_handler;
	struct input_handle pen_handle;
	struct delayed_work hall_sensor_work;
	struct delayed_work hall_sensor_dowork;
}* hall_sensor_dev;

static BLOCKING_NOTIFIER_HEAD(pen_detection_notifier_list);

/**
 * pen_detection_register_client - register a client notifier
 * @nb: notifier block to callback on events
 *
 * This function registers a notifier callback function
 * to pen_detection_notifier_list, which would be called when
 * the passive pen is inserted or pulled out.
 */
int pen_detection_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pen_detection_notifier_list,
						nb);
}
EXPORT_SYMBOL(pen_detection_register_client);

/**
 * pen_detection_unregister_client - unregister a client notifier
 * @nb: notifier block to callback on events
 *
 * This function unregisters the callback function from
 * pen_detection_notifier_list.
 */
int pen_detection_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&pen_detection_notifier_list,
						  nb);
}
EXPORT_SYMBOL(pen_detection_unregister_client);

/**
 * pen_detection_notifier_call_chain - notify clients of pen_detection_events
 * @val: event PEN_DETECTION_INSERT or PEN_DETECTION_PULL
 * @v: notifier data, inculde display pen detection event.
 */
static int pen_detection_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&pen_detection_notifier_list, val,
					    v);
}

//static void hall_sensor_shutdown(struct platform_device *pdev);
int pen_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id){
	pen_dbg_func_in();
	pen_info("[%s] hall_sensor connect to handler\n", DRIVER_NAME);
	return 0;
}

void pen_event(struct input_handle *handle, unsigned int type, unsigned int code, int value){
	pen_dbg_func_in();
	if(type==EV_SW && code==SW_PEN_INSERTED ){
		if(value != 2 && !!test_bit(code, hall_sensor_dev->pen_indev->sw) != !hall_sensor_dev->status){
			__change_bit(code,  hall_sensor_dev->pen_indev->sw);
			pen_info("[%s] reset dev->sw=%d \n", DRIVER_NAME,!hall_sensor_dev->status);
		}
	}
}

bool pen_match(struct input_handler *handler, struct input_dev *dev){
	pen_dbg_func_in();
	if( (dev->name && handler->name) &&
		!strcmp(dev->name,"pen_input") &&
		!strcmp(handler->name,"pen_input_handler"))
		    return true;

	return false;
}

static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	pen_dbg_func_in();
	if(!hall_sensor_dev)
		return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->status);
}
static ssize_t store_action_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	unsigned long flags;

	pen_dbg_func_in();
	if(!hall_sensor_dev){
	    pen_err("Hall sensor does not exist!\n");
	    return 0;
	    //return sprintf(buf, "Hall sensor does not exist!\n");
	}

	sscanf(buf, "%du", &request);
	spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
	if (!request)
		hall_sensor_dev->status = 0;
	else
		hall_sensor_dev->status = 1;
	spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	pen_info("[%s] SW_PEN_INSERTED rewite value = %d\n", DRIVER_NAME,!hall_sensor_dev->status);
	return count;
}

static ssize_t show_hall_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	pen_dbg_func_in();
	if(!hall_sensor_dev)
		return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->enable);
}

static ssize_t store_hall_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;

	pen_dbg_func_in();
	if(!hall_sensor_dev){
	    pen_err("Hall sensor does not exist!\n");
	    return 0;
	    //return sprintf(buf, "Hall sensor does not exist!\n");
	}
	sscanf(buf, "%du", &request);
	if(request==hall_sensor_dev->enable){
		return count;
	}
	else {
		unsigned long flags;
		spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
		if (hall_sensor_dev->enable==0){
			enable_irq(hall_sensor_dev->irq);
			hall_sensor_dev->enable=1;
		}
		else if (hall_sensor_dev->enable==1){
			disable_irq(hall_sensor_dev->irq);
			hall_sensor_dev->enable=0;
		}
		spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	}
	return count;
}


static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO|S_IWUSR, show_action_status, store_action_status, 0, 0);
static SENSOR_DEVICE_ATTR_2(activity, S_IRUGO|S_IWUSR,show_hall_sensor_enable, store_hall_sensor_enable, 0, 0);

static struct attribute *hall_sensor_attrs[] = {
	&sensor_dev_attr_action_status.dev_attr.attr,
	&sensor_dev_attr_activity.dev_attr.attr,
	NULL
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_sensor",
	.attrs = hall_sensor_attrs
};

static int pen_input_device_create(void)
{
	int err = 0;

	pen_dbg_func_in();
	hall_sensor_dev->pen_indev = input_allocate_device();
	if(!hall_sensor_dev->pen_indev){
		pen_err("[%s] pen_indev allocation fails\n", DRIVER_NAME);
		err = -ENOMEM;
		goto exit;
	}

	hall_sensor_dev->pen_indev->name = "pen_detect";
	hall_sensor_dev->pen_indev->phys= "/dev/input/pen_detect";
	hall_sensor_dev->pen_indev->dev.parent= NULL;
	input_set_capability(hall_sensor_dev->pen_indev, EV_SW, SW_PEN_INSERTED);

	err = input_register_device(hall_sensor_dev->pen_indev);
	if (err) {
		pen_err("[%s] input registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_input_free;
	}
	hall_sensor_dev->pen_handler.match=pen_match;
	hall_sensor_dev->pen_handler.connect=pen_connect;
	hall_sensor_dev->pen_handler.event=pen_event;
	hall_sensor_dev->pen_handler.name="pen_detect_handler";
	hall_sensor_dev->pen_handler.id_table = mID;
	err=input_register_handler(& hall_sensor_dev->pen_handler);
	if(err){
		pen_err("[%s] handler registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_unregister_input_dev;
	}
	hall_sensor_dev->pen_handle.name="pen_detect_handle";
	hall_sensor_dev->pen_handle.open=1;         //receive any event from hall sensor
	hall_sensor_dev->pen_handle.dev=hall_sensor_dev->pen_indev;
	hall_sensor_dev->pen_handle.handler=&hall_sensor_dev->pen_handler;
	err=input_register_handle(& hall_sensor_dev->pen_handle);
	if(err){
		pen_err("[%s] handle registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_unregister_handler;
	}
	return 0;

exit_unregister_handler:
	input_unregister_handler(& hall_sensor_dev->pen_handler);
exit_unregister_input_dev:
	input_unregister_device(hall_sensor_dev->pen_indev);
exit_input_free:
	input_free_device(hall_sensor_dev->pen_indev);
	hall_sensor_dev->pen_indev = NULL;
exit:
	return err;
}



static void pen_do_work_function(struct work_struct *dat)
{
	pen_dbg_func_in();
	pen_info("[%s] hall_sensor_interrupt = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	cancel_delayed_work(&hall_sensor_dev->hall_sensor_work);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
}


static void pen_report_function(struct work_struct *dat)
{
	unsigned long flags;
	int counter, status = 0, initial_status;

	pen_dbg_func_in();
	if (!hall_sensor_dev) {
		pen_info("[%s] hall_sensor_dev isn't init.\n", DRIVER_NAME);
		return;
	}
	initial_status = hall_sensor_dev->status;
	for (counter = 0;counter < 3;counter++){
		msleep(50);
		spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
		if (gpio_get_value(hall_sensor_dev->gpio) > 0){
			hall_sensor_dev->status = 1;
			status++;
		}else{
			hall_sensor_dev->status = 0;
		}
		pen_info("[%s] SW_PEN_INSERTED check[%d] value = %d\n", DRIVER_NAME, counter, !hall_sensor_dev->status);
		spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	}

	if((status > 0) && (status < 3)){
		pen_info("[%s] SW_PEN_INSERTED do not report to framework.\n", DRIVER_NAME);
		hall_sensor_dev->status = initial_status;
		return;
	}

	if (!hall_sensor_dev->pen_indev) {
		pen_info("[%s] pen_indev isn't ready,don't report.\n", DRIVER_NAME);
		return;
	}
	input_report_switch(hall_sensor_dev->pen_indev, SW_PEN_INSERTED, !hall_sensor_dev->status);
	input_sync(hall_sensor_dev->pen_indev);
#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&hall_sensor_dev->wake_lock);
#else
#endif

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_timeout(&hall_sensor_dev->wake_lock, msecs_to_jiffies(3000));
#else
	PM_WAKEUP_EVENT(hall_sensor_dev->wake_lock,msecs_to_jiffies(3000));
#endif

	if(!hall_sensor_dev->status)
		pen_detection_notifier_call_chain(PEN_DETECTION_INSERT, NULL);
	else
		pen_detection_notifier_call_chain(PEN_DETECTION_PULL, NULL);
	pen_info("[%s] SW_pen report value = %d\n", DRIVER_NAME,!hall_sensor_dev->status);
}

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	pen_dbg_func_in();
	pen_info("[%s] hall_sensor_interrupt_handler = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	queue_delayed_work(hall_sensor_do_wq, &hall_sensor_dev->hall_sensor_dowork, msecs_to_jiffies(0));
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock(&hall_sensor_dev->wake_lock);
#else
#endif
	return IRQ_HANDLED;
}

static int set_irq_hall_sensor(void)
{
	int rc = 0 ;

	pen_dbg_func_in();
	hall_sensor_dev->irq = gpio_to_irq(hall_sensor_dev->gpio);
	pen_info("[%s] hall_sensor irq = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	rc = request_irq(hall_sensor_dev->irq,hall_sensor_interrupt_handler,
			IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,"hall_sensor_irq",hall_sensor_dev);
	if (rc<0) {
		pen_info("[%s] Could not register for hall sensor interrupt, irq = %d, rc = %d\n", DRIVER_NAME,hall_sensor_dev->irq,rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(hall_sensor_dev->irq);

	return 0;

err_gpio_request_irq_fail:
	return rc;
}

#ifdef CONFIG_OF
static const struct of_device_id bu520xx_pen_match[] = {
	{ .compatible = "rohm,bu520xx_pen_detect", },
	{}
};
#else
#define bu520xx_pen_match NULL
#endif
MODULE_DEVICE_TABLE(of, bu520xx_pen_match);

static struct platform_driver bu520xx_pen_driver = {
	.probe		= bu520xx_pen_probe,
	.remove		= bu520xx_pen_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.of_match_table    = bu520xx_pen_match,
	},
};

static int bu520xx_pen_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	enum of_gpio_flags flags;
	int ret;

	pen_dbg_func_in();

	//set file node
	hall_sensor_kobj = kobject_create_and_add("hall_sensor_kobject", kernel_kobj);
	if (!hall_sensor_kobj){
		pen_err("[%s] hall_sensor_kobject fails for hall sensor\n", DRIVER_NAME);
		platform_device_unregister(pdev);
		platform_driver_unregister(&bu520xx_pen_driver);
		return -ENOMEM;
	}

	ret = sysfs_create_group(hall_sensor_kobj, &hall_sensor_group);
	if (ret){
		goto fail_for_hall_sensor;
	}

	//Memory allocation
	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
		pen_err("[%s] Memory allocation fails for hall sensor\n", DRIVER_NAME);
		ret = -ENOMEM;
		goto fail_for_hall_sensor;
	}

	spin_lock_init(&hall_sensor_dev->mHallSensorLock);
	hall_sensor_dev->enable = 1;

	//set gpio
	hall_sensor_dev->gpio = of_get_named_gpio_flags(np,
			"rohm,nirq-gpio", 0, &flags);
	 if (hall_sensor_dev->gpio < 0)
	{
		pen_err("[%s] GPIO for hall sensor does not exist.\n", DRIVER_NAME);
		ret= -1;
		goto fail_for_set_gpio_hall_sensor;
	}

	gpio_request(hall_sensor_dev->gpio,"pen_detect_gpio");
	gpio_direction_input(hall_sensor_dev->gpio);

	//create input_dev
	hall_sensor_dev->pen_indev = NULL;
	ret = pen_input_device_create();
	if (ret < 0)
		goto fail_for_create_input_dev;

	//init workqueue & start detect signal
	hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
	hall_sensor_do_wq = create_singlethread_workqueue("hall_sensor_do_wq");
	INIT_DELAYED_WORK(&hall_sensor_dev->hall_sensor_work, pen_report_function);
	INIT_DELAYED_WORK(&hall_sensor_dev->hall_sensor_dowork, pen_do_work_function);


	queue_delayed_work(hall_sensor_do_wq, &hall_sensor_dev->hall_sensor_dowork, 0);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&hall_sensor_dev->wake_lock, WAKE_LOCK_SUSPEND, "pen_suspend_blocker");
#else
	PM_WAKEUP_REGISTER(&pdev->dev, hall_sensor_dev->wake_lock, "pen_suspend_blocker");
	if(!hall_sensor_dev->wake_lock){
		dev_err(&pdev->dev,"%s: Failed to allocate wakeup source\n",__func__);
		ret = -ENOMEM;
		goto fail_wakeup_init;
	}
#endif
	//set irq
	ret = set_irq_hall_sensor();
	if (ret < 0)
		goto fail_for_irq_hall_sensor;

	pen_info("hall_sensor_init Done.\r\n");
	return 0;

fail_for_irq_hall_sensor:
	destroy_workqueue(hall_sensor_do_wq);
	destroy_workqueue(hall_sensor_wq);
	input_free_device(hall_sensor_dev->pen_indev);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&hall_sensor_dev->wake_lock);
#else
	PM_WAKEUP_UNREGISTER(hall_sensor_dev->wake_lock);
#endif
fail_wakeup_init:
fail_for_create_input_dev:
    hall_sensor_dev->pen_indev=NULL;
	gpio_free(hall_sensor_dev->gpio);
fail_for_set_gpio_hall_sensor:
	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;

fail_for_hall_sensor:
	kobject_put(hall_sensor_kobj);
	return ret;
}

static int bu520xx_pen_remove(struct platform_device *pdev)
{
	disable_irq(hall_sensor_dev->irq);
	gpio_free(hall_sensor_dev->gpio);
	destroy_workqueue(hall_sensor_do_wq);
	destroy_workqueue(hall_sensor_wq);
	input_free_device(hall_sensor_dev->pen_indev);
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_destroy(&hall_sensor_dev->wake_lock);
#else
	PM_WAKEUP_UNREGISTER(hall_sensor_dev->wake_lock);
#endif
	hall_sensor_dev->pen_indev=NULL;
	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;
	kobject_put(hall_sensor_kobj);
	platform_driver_unregister(&bu520xx_pen_driver);
	platform_device_unregister(pdev);
	return 1;
}


module_platform_driver(bu520xx_pen_driver);


MODULE_DESCRIPTION("Rohm_Hall_sensor-bu520xx_pen Driver");
MODULE_AUTHOR("huangjq9 <h@lenovo.com>");
MODULE_LICENSE("GPL v2");
