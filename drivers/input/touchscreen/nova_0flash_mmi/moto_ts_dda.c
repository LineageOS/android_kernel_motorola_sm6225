#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/uaccess.h>
#include "moto_ts_dda.h"

//cdev registration
#define DRIVER_NAME "moto_dda_penraw_driver"
#define DEVICE_NAME "moto_penraw"
#define MINOR_NUMBER_START ((const unsigned int)0) /* the minor number starts at */
#define NUMBER_MINOR_NUMBER ((const unsigned int)1) /* the number of minor numbers */
#define DDA_MAX_BUFFER 32
#define MAX_IO_CONTROL_REPORT 16
#define DDA_TOUCH_ID_MAX 10
#define DDA_DEVICE_NAME_MAXLEN	32

static bool debug_log_flag = false;
#define DDA_INFO(fmt, args...)    pr_info("[MOTO_DDA_INFO] %s %d: " fmt, __func__, __LINE__, ##args)
#define DDA_ERR(fmt, args...)    pr_err("[MOTO_DDA_ERR] %s %d: " fmt, __func__, __LINE__, ##args)
#define DDA_DBG(fmt, args...)    {if(debug_log_flag)pr_debug("[MOTO_DDA_DBG] %s %d: " fmt, __func__, __LINE__, ##args);}


struct dda_io_device_name {
	char device_name[DDA_DEVICE_NAME_MAXLEN];
};

#ifdef MOTO_DDA_PASSIVE_STYLUS
enum {
	DDA_FINGER_RELEASE = 0,
	DDA_FINGER_ENTER  = 1,
	DDA_FINGER_MOVE  = 2
};

struct dda_finger_info {
	unsigned char frame_no;
	unsigned char reserve[3];
	struct dda_finger_coords coords;
};

struct dda_io_finger_reports {
	unsigned char report_num;
	unsigned char reserve[3];
	struct dda_finger_info finger_info[MAX_IO_CONTROL_REPORT];
};

struct dda_finger_report_buffer{
	unsigned char finger_report_num;
	unsigned char frame_no;
	unsigned char buffer_wp;
	unsigned char reserve[1];
	unsigned char last_status;
	struct dda_finger_info finger_report_buffer[DDA_MAX_BUFFER];
};
#endif //MOTO_DDA_PASSIVE_STYLUS


#ifdef MOTO_DDA_ACTIVE_STYLUS
/* for DDA goodix_penraw */
enum{
	DATA_TYPE_RAW = 0
};

struct dda_pen_info {
	unsigned char frame_no;
	unsigned char data_type;
	unsigned char reserve[2];
	struct dda_pen_coords coords;
};

struct dda_io_pen_reports {
	unsigned char report_num;
	unsigned char reserve[3];
	struct dda_pen_info pen_info[MAX_IO_CONTROL_REPORT];
};

struct dda_pen_report_buffer{
	unsigned char pen_report_num;
	unsigned char frame_no;
	unsigned char buffer_wp;
	unsigned char reserve[1];
	struct dda_pen_info pen_report_buffer[DDA_MAX_BUFFER];
};
#endif //MOTO_DDA_ACTIVE_STYLUS

struct dda_cdev_control{
	unsigned int major_number; /* the major number of the device */
	struct cdev penraw_char_dev; /* character device */
	struct class* penraw_char_dev_class; /* class object */
};


//DDA cdev
static struct dda_cdev_control dda_cdev_ctrl;

#ifdef MOTO_DDA_PASSIVE_STYLUS
//DDA raw finger report buffer
static struct dda_finger_report_buffer dda_finger_report_buf[DDA_TOUCH_ID_MAX];
static int dda_finger_id_assign_table[DDA_TOUCH_ID_MAX];
static int dda_finger_id_last_assign_table[DDA_TOUCH_ID_MAX];
#endif //MOTO_DDA_PASSIVE_STYLUS

#ifdef MOTO_DDA_ACTIVE_STYLUS
//DDA raw pen report buffer
static struct dda_pen_report_buffer dda_pen_report_buf;
#endif //MOTO_DDA_ACTIVE_STYLUS

static spinlock_t lock;
static int open_count = 0;

static char dda_device_name[DDA_DEVICE_NAME_MAXLEN];


//prototype of private functions
#ifdef MOTO_DDA_PASSIVE_STYLUS
static int get_finger_report(int id, unsigned long arg);
#endif //MOTO_DDA_PASSIVE_STYLUS
#ifdef MOTO_DDA_ACTIVE_STYLUS
static int get_pen_report(unsigned long arg);
#endif //MOTO_DDA_ACTIVE_STYLUS

static int get_device_name(unsigned long arg);

// Linux 2.0/2.2
static int penraw_open(struct inode * inode, struct file * file)
{
	spin_lock(&lock);
	if (open_count) {
		spin_unlock(&lock);
		return -EBUSY;
	}
	open_count++;
	spin_unlock(&lock);

	return 0;
}

// Linux 2.1: int type
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 1, 0)
static int penraw_close(struct inode * inode, struct file * file)
{
	spin_lock(&lock);
	open_count--;
	spin_unlock(&lock);

	return 0;
}
#else
static void penraw_close(struct inode * inode, struct file * file)
{
	spinlock(&lock);
	open_count--;
	spin_unlock(&lock);
}
#endif

#define PENRAW_IOC_TYPE 'P'
#ifdef MOTO_DDA_ACTIVE_STYLUS
#define PENRAW_GET_VALUES _IOR(PENRAW_IOC_TYPE, 0, struct dda_io_pen_reports)

static struct dda_io_pen_reports pen_reports; // return report
#endif //MOTO_DDA_ACTIVE_STYLUS

#ifdef MOTO_DDA_PASSIVE_STYLUS
#define PENRAW_GET_VALUES_ID0 _IOR(PENRAW_IOC_TYPE, 1, struct dda_io_finger_reports)
#define PENRAW_GET_VALUES_ID1 _IOR(PENRAW_IOC_TYPE, 2, struct dda_io_finger_reports)
#define PENRAW_GET_VALUES_ID2 _IOR(PENRAW_IOC_TYPE, 3, struct dda_io_finger_reports)
#define PENRAW_GET_VALUES_ID3 _IOR(PENRAW_IOC_TYPE, 4, struct dda_io_finger_reports)
#define PENRAW_GET_VALUES_ID4 _IOR(PENRAW_IOC_TYPE, 5, struct dda_io_finger_reports)
#define PENRAW_GET_VALUES_ID5 _IOR(PENRAW_IOC_TYPE, 6, struct dda_io_finger_reports)
#define PENRAW_GET_VALUES_ID6 _IOR(PENRAW_IOC_TYPE, 7, struct dda_io_finger_reports)
#define PENRAW_GET_VALUES_ID7 _IOR(PENRAW_IOC_TYPE, 8, struct dda_io_finger_reports)
#define PENRAW_GET_VALUES_ID8 _IOR(PENRAW_IOC_TYPE, 9, struct dda_io_finger_reports)
#define PENRAW_GET_VALUES_ID9 _IOR(PENRAW_IOC_TYPE, 10, struct dda_io_finger_reports)

static struct dda_io_finger_reports finger_reports; // return report
#endif //MOTO_DDA_PASSIVE_STYLUS

#define PENRAW_GET_DEVICE_NAME _IOR(PENRAW_IOC_TYPE, 11, struct dda_io_device_name)

static long penraw_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
#ifdef MOTO_DDA_ACTIVE_STYLUS		
		case PENRAW_GET_VALUES:
			ret = get_pen_report(arg);
			break;
#endif
#ifdef MOTO_DDA_PASSIVE_STYLUS		
		case PENRAW_GET_VALUES_ID0:
			ret = get_finger_report(0,arg);
			break;
		case PENRAW_GET_VALUES_ID1:
			ret = get_finger_report(1,arg);
			break;
		case PENRAW_GET_VALUES_ID2:
			ret = get_finger_report(2,arg);
			break;
		case PENRAW_GET_VALUES_ID3:
			ret = get_finger_report(3,arg);
			break;
		case PENRAW_GET_VALUES_ID4:
			ret = get_finger_report(4,arg);
			break;
		case PENRAW_GET_VALUES_ID5:
			ret = get_finger_report(5,arg);
			break;
		case PENRAW_GET_VALUES_ID6:
			ret = get_finger_report(6,arg);
			break;
		case PENRAW_GET_VALUES_ID7:
			ret = get_finger_report(7,arg);
			break;
		case PENRAW_GET_VALUES_ID8:
			ret = get_finger_report(8,arg);
			break;
		case PENRAW_GET_VALUES_ID9:
			ret = get_finger_report(9,arg);
			break;
#endif //MOTO_DDA_PASSIVE_STYLUS
		case PENRAW_GET_DEVICE_NAME:
			ret = get_device_name(arg);
			break;
		default:
			DDA_ERR("unsupported command %d", cmd);
			return -EFAULT;
	}
	return ret;
}

#ifdef MOTO_DDA_PASSIVE_STYLUS
static int get_finger_report(int id, unsigned long arg){
	unsigned long flags;
	struct dda_finger_info *pfinger_info;
	unsigned char cnt;
	unsigned char finger_buffer_rp;
	unsigned char wp;
	unsigned char num;
	int touch_id;
	//enter to critical session
	local_irq_save(flags);
	touch_id = dda_finger_id_assign_table[id];
	if(-1 == touch_id){
		touch_id = dda_finger_id_last_assign_table[id];
		if(-1 == touch_id){
			DDA_ERR("PassiveStylusDDA touch_id is not assignd for %d",id);
			touch_id = id;
		}
	}

	wp = dda_finger_report_buf[touch_id].buffer_wp;
	num = dda_finger_report_buf[touch_id].finger_report_num;
	if(MAX_IO_CONTROL_REPORT <= num) {
		finger_buffer_rp = (unsigned char)((wp + (DDA_MAX_BUFFER -
		MAX_IO_CONTROL_REPORT)) % DDA_MAX_BUFFER);
	} else {
		num = MAX_IO_CONTROL_REPORT;	//for safety implementation
		finger_buffer_rp = 0;
	}
	memset(&finger_reports, 0, sizeof(finger_reports));
	finger_reports.report_num = num;
	pfinger_info = (struct dda_finger_info *)&finger_reports.finger_info[0];
	for(cnt = 0; cnt < num; cnt++) {
		memcpy(pfinger_info, &dda_finger_report_buf[touch_id].finger_report_buffer[finger_buffer_rp],
		sizeof(struct dda_finger_info));
		pfinger_info++;
		finger_buffer_rp++;
		if(DDA_MAX_BUFFER == finger_buffer_rp) {
			finger_buffer_rp = 0;
		}
	}
	//leave from critical session
	local_irq_restore(flags);
	
	
	if (copy_to_user((void __user *)arg, &finger_reports, sizeof(finger_reports))) {
		return -EFAULT;
	}
	return 0;
}
#endif //MOTO_DDA_PASSIVE_STYLUS

#ifdef MOTO_DDA_ACTIVE_STYLUS
static int get_pen_report(unsigned long arg){
	unsigned long flags;
	struct dda_pen_info *ppen_info;
	unsigned char cnt;
	unsigned char pen_buffer_rp;
	unsigned char wp;
	unsigned char num;

	local_irq_save(flags);
	wp = dda_pen_report_buf.buffer_wp;
	num = dda_pen_report_buf.pen_report_num;
	local_irq_restore(flags);
	if(MAX_IO_CONTROL_REPORT <= num) {
		pen_buffer_rp = (unsigned char)((wp + (DDA_MAX_BUFFER -
			MAX_IO_CONTROL_REPORT)) % DDA_MAX_BUFFER);
	} else {
		pen_buffer_rp = 0;
	}
	memset(&pen_reports, 0, sizeof(pen_reports));
	pen_reports.report_num = num;
	ppen_info = (struct dda_pen_info *)&pen_reports.pen_info[0];
	for(cnt = 0; cnt < num; cnt++) {
		memcpy(ppen_info, &dda_pen_report_buf.pen_report_buffer[pen_buffer_rp],
			sizeof(struct dda_pen_info));
		ppen_info++;
		pen_buffer_rp++;
		if(DDA_MAX_BUFFER == pen_buffer_rp) {
			pen_buffer_rp = 0;
		}
	}
	if (copy_to_user((void __user *)arg, &pen_reports, sizeof(pen_reports))) {
		return -EFAULT;
	}
	return 0;
}
#endif //MOTO_DDA_ACTIVE_STYLUS

static int get_device_name(unsigned long arg){
	if (copy_to_user((void __user *)arg, &dda_device_name, sizeof(dda_device_name))) {
		return -EFAULT;
	}
	return 0;
}

static struct file_operations penraw_fops = {
	.owner = THIS_MODULE,
	.open = penraw_open,
	.release = penraw_close,
	.unlocked_ioctl = penraw_ioctl,
};

/* functions for DDA Features */
#ifdef MOTO_DDA_PASSIVE_STYLUS
void moto_dda_process_finger_press(uint8_t touch_id, struct dda_finger_coords *finger_data) {
	struct dda_finger_info *pfinger_info;
	int i;
	unsigned char id_is_found;

	if(touch_id >= DDA_TOUCH_ID_MAX){
		return;
	}

	// ioctl-DAA Buffering finger raw data
	pfinger_info  = &dda_finger_report_buf[touch_id].finger_report_buffer[0];
	pfinger_info += dda_finger_report_buf[touch_id].buffer_wp;
	memset(pfinger_info, 0, sizeof(struct dda_finger_info));
	pfinger_info->coords.x = finger_data->x;
	pfinger_info->coords.y = finger_data->y;
	pfinger_info->coords.p = finger_data->p;
	pfinger_info->coords.minor = finger_data->minor;
	pfinger_info->coords.major = finger_data->major;
	pfinger_info->frame_no = dda_finger_report_buf[touch_id].frame_no;
	DDA_DBG("P:[%d] %d %d %d (%d)\n",touch_id, finger_data->x, finger_data->y, finger_data->p, dda_finger_report_buf[touch_id].frame_no);
	dda_finger_report_buf[touch_id].frame_no++;
	if(DDA_FINGER_RELEASE == dda_finger_report_buf[touch_id].last_status){
		pfinger_info->coords.status =  DDA_FINGER_ENTER;
		//check registerd iDs  to avoid duplicated registration.
		id_is_found = 0;
		for(i=0;i<DDA_TOUCH_ID_MAX;i++){
			if(touch_id == dda_finger_id_assign_table[i]){
				id_is_found = 1;
				break;
			}
		}
		if(0 == id_is_found){
			for(i=0;i<DDA_TOUCH_ID_MAX;i++){
				if(-1 == dda_finger_id_assign_table[i]){
					dda_finger_id_assign_table[i] = touch_id;
					break;
				}
			}
		}
	}else{
		pfinger_info->coords.status =  DDA_FINGER_MOVE;
	}
	dda_finger_report_buf[touch_id].last_status = pfinger_info->coords.status;
	if(MAX_IO_CONTROL_REPORT > dda_finger_report_buf[touch_id].finger_report_num) {
		// Max count: MAX_IO_CONTROL_REPORT
		dda_finger_report_buf[touch_id].finger_report_num++;
	}
	dda_finger_report_buf[touch_id].buffer_wp++;
	if(DDA_MAX_BUFFER == dda_finger_report_buf[touch_id].buffer_wp) {
		dda_finger_report_buf[touch_id].buffer_wp = 0;
	}
}

void moto_dda_process_finger_release(uint8_t touch_id) {
	struct dda_finger_info *pfinger_info;
	int i;

	if(touch_id >= DDA_TOUCH_ID_MAX){
		return;
	}

	// ioctl-DAA Buffering finger raw data
	pfinger_info  = &dda_finger_report_buf[touch_id].finger_report_buffer[0];
	pfinger_info += dda_finger_report_buf[touch_id].buffer_wp;
	memset(pfinger_info, 0, sizeof(struct dda_finger_info));
	pfinger_info->coords.x = 0xFFFFFFFF;
	pfinger_info->coords.y = 0xFFFFFFFF;
	pfinger_info->coords.p = 0;
	pfinger_info->coords.minor = 0;
	pfinger_info->coords.major = 0;
	pfinger_info->frame_no = dda_finger_report_buf[touch_id].frame_no;
	DDA_DBG("R:[%d] (%d)\n",touch_id, dda_finger_report_buf[touch_id].frame_no);
	dda_finger_report_buf[touch_id].frame_no++;
	pfinger_info->coords.status =  DDA_FINGER_RELEASE;
	dda_finger_report_buf[touch_id].last_status = pfinger_info->coords.status;
	for(i=0;i<DDA_TOUCH_ID_MAX;i++){
		if(touch_id == dda_finger_id_assign_table[i]){
			dda_finger_id_last_assign_table[i] = dda_finger_id_assign_table[i];
			dda_finger_id_assign_table[i] = -1;
			break;
		}

	}
	if(MAX_IO_CONTROL_REPORT > dda_finger_report_buf[touch_id].finger_report_num) {
		// Max count: MAX_IO_CONTROL_REPORT
		dda_finger_report_buf[touch_id].finger_report_num++;
	}
	dda_finger_report_buf[touch_id].buffer_wp++;
	if(DDA_MAX_BUFFER == dda_finger_report_buf[touch_id].buffer_wp) {
		dda_finger_report_buf[touch_id].buffer_wp = 0;
	}

}
#endif //MOTO_DDA_PASSIVE_STYLUS

#ifdef MOTO_DDA_ACTIVE_STYLUS
void moto_dda_process_pen_report(struct dda_pen_coords *pen_data) {
	struct dda_pen_info *ppen_info;

	if (pen_data->status) {
		// ioctl-DAA Buffering pen raw data
		ppen_info  = &dda_pen_report_buf.pen_report_buffer[0];
		ppen_info += dda_pen_report_buf.buffer_wp;
		memset(ppen_info, 0, sizeof(struct dda_pen_info));
		ppen_info->coords.status = (signed char)pen_data->status;
		ppen_info->coords.tool_type = (signed char)pen_data->tool_type;
		ppen_info->coords.tilt_x = pen_data->tilt_x;
		ppen_info->coords.tilt_y = pen_data->tilt_y;
		ppen_info->coords.x = pen_data->x;
		ppen_info->coords.y = pen_data->y;
		ppen_info->coords.p = pen_data->p;
		ppen_info->frame_no = dda_pen_report_buf.frame_no;
		ppen_info->data_type = DATA_TYPE_RAW;
		DDA_DBG("S:[%d] %d %d %d (%d)\n", pen_data->status, (int)pen_data->x, (int)pen_data->y, (int)pen_data->p, dda_pen_report_buf.frame_no);
		dda_pen_report_buf.frame_no++;
		if(MAX_IO_CONTROL_REPORT > dda_pen_report_buf.pen_report_num) {
			// Max count: MAX_IO_CONTROL_REPORT
			dda_pen_report_buf.pen_report_num++;
		}
		dda_pen_report_buf.buffer_wp++;
		if(DDA_MAX_BUFFER == dda_pen_report_buf.buffer_wp) {
			dda_pen_report_buf.buffer_wp = 0;
		}
	}
}
#endif //MOTO_DDA_ACTIVE_STYLUS


void moto_dda_init(char *device_name) {
#ifdef MOTO_DDA_PASSIVE_STYLUS
	int cnt;
#endif //MOTO_DDA_PASSIVE_STYLUS

	memset(dda_device_name,0x00,sizeof(dda_device_name));
	strncpy(dda_device_name,device_name,sizeof(dda_device_name)-1);

#ifdef MOTO_DDA_PASSIVE_STYLUS
	for(cnt =0;cnt<DDA_TOUCH_ID_MAX;cnt++ ){
		dda_finger_report_buf[cnt].finger_report_num = 0;
		dda_finger_report_buf[cnt].frame_no = 0;
		dda_finger_report_buf[cnt].buffer_wp = 0;
		dda_finger_report_buf[cnt].last_status = DDA_FINGER_RELEASE;
		dda_finger_id_assign_table[cnt] = -1;
		dda_finger_id_last_assign_table[cnt] = -1;
	}
#endif //MOTO_DDA_PASSIVE_STYLUS

#ifdef MOTO_DDA_ACTIVE_STYLUS
	dda_pen_report_buf.pen_report_num = 0;
	dda_pen_report_buf.frame_no = 0;
	dda_pen_report_buf.buffer_wp = 0;
#endif //MOTO_DDA_ACTIVE_STYLUS

	memset(&dda_cdev_ctrl.penraw_char_dev, 0, sizeof(dda_cdev_ctrl.penraw_char_dev));
	dda_cdev_ctrl.penraw_char_dev_class = NULL;
	spin_lock_init(&lock);
}

int moto_dda_register_cdevice(void) {
	int ret;
	int alloc_ret;
	int cdev_err;
	dev_t dev;
	struct device *penraw_dev;

	DDA_INFO("moto stylus dda register cdevice start");
	/* get not assigned major numbers */
	alloc_ret = alloc_chrdev_region(&dev, MINOR_NUMBER_START, NUMBER_MINOR_NUMBER,
		DRIVER_NAME);
	if (alloc_ret != 0) {
		DDA_ERR("failed to alloc_chrdev_region()");
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		return alloc_ret;
	}

	/* get one number from the not-assigend numbers */
	dda_cdev_ctrl.major_number = MAJOR(dev);

	/* initialize cdev and function table */
	cdev_init(&dda_cdev_ctrl.penraw_char_dev, &penraw_fops);
	dda_cdev_ctrl.penraw_char_dev.owner = THIS_MODULE;

	/* register the driver */
	cdev_err = cdev_add(&dda_cdev_ctrl.penraw_char_dev, dev, NUMBER_MINOR_NUMBER);
	if (cdev_err != 0) {
		DDA_ERR(KERN_ERR "failed to cdev_add()");
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		return cdev_err;
	}

	/* register a class */
	dda_cdev_ctrl.penraw_char_dev_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(dda_cdev_ctrl.penraw_char_dev_class)) {
		DDA_ERR("class_create()");
		cdev_del(&dda_cdev_ctrl.penraw_char_dev);
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		ret = PTR_ERR(dda_cdev_ctrl.penraw_char_dev_class);
		dda_cdev_ctrl.penraw_char_dev_class = NULL;
		//return error code
		return ret;
	}

	/* create devive /dev/moto_penraw*/
	penraw_dev = device_create(dda_cdev_ctrl.penraw_char_dev_class, NULL,
		MKDEV(dda_cdev_ctrl.major_number, 0), NULL, DEVICE_NAME);
	if (IS_ERR(penraw_dev)) {
		DDA_ERR("device_create()");
		class_destroy(dda_cdev_ctrl.penraw_char_dev_class);
		dda_cdev_ctrl.penraw_char_dev_class = NULL;
		cdev_del(&dda_cdev_ctrl.penraw_char_dev);
		unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);
		ret = PTR_ERR(penraw_dev);
		penraw_dev = NULL;
		return ret;
	}

	DDA_INFO("moto stylus dda register cdevice success");
	return 0;
}

void moto_dda_exit(void) {
	dev_t dev = MKDEV(dda_cdev_ctrl.major_number, MINOR_NUMBER_START);

	/* remove "/dev/moto_penraw" */
	if (NULL != dda_cdev_ctrl.penraw_char_dev_class) {
		device_destroy(dda_cdev_ctrl.penraw_char_dev_class,
			MKDEV(dda_cdev_ctrl.major_number, 0));
	}

	/* remove class */
	if (NULL != dda_cdev_ctrl.penraw_char_dev_class) {
		class_destroy(dda_cdev_ctrl.penraw_char_dev_class);
	}

	/* remove driver */
	cdev_del(&dda_cdev_ctrl.penraw_char_dev);

	/* release the major number */
	unregister_chrdev_region(dev, NUMBER_MINOR_NUMBER);

	DDA_INFO("moto_stylus_dda_exit success");
}

