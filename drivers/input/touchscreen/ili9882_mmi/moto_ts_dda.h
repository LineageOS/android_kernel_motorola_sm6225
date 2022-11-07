
#define MOTO_DDA_PASSIVE_STYLUS
//#define MOTO_DDA_ACTIVE_STYLUS


#ifdef MOTO_DDA_PASSIVE_STYLUS
struct dda_finger_coords {
	unsigned char status;	//Finger release, Finger enter or Finger move
	unsigned char reserve[3];
	unsigned int x;
	unsigned int y;
	unsigned int p;
	unsigned int minor;
	unsigned int major;
};
#endif	//MOTO_DDA_PASSIVE_STYLUS


#ifdef MOTO_DDA_ACTIVE_STYLUS
struct dda_pen_coords {
	signed char status;
	signed char tool_type;
	signed char tilt_x;
	signed char tilt_y;
	unsigned long int x;
	unsigned long int y;
	unsigned long int p;
};
#endif //MOTO_DDA_ACTIVE_STYLUS



void moto_dda_init(char *device_name);
void moto_dda_exit(void);
int moto_dda_register_cdevice(void);

#ifdef MOTO_DDA_PASSIVE_STYLUS
void moto_dda_process_finger_press(uint8_t touch_id, struct dda_finger_coords *finger_data);
void moto_dda_process_finger_release(uint8_t touch_id);
#endif	//MOTO_DDA_PASSIVE_STYLUS

#ifdef MOTO_DDA_ACTIVE_STYLUS
void moto_dda_process_pen_report(struct dda_pen_coords *pen_data);
#endif //MOTO_DDA_ACTIVE_STYLUS
