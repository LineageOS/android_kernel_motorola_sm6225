#ifndef HAPTIC_NV_DTS_H
#define HAPTIC_NV_DTS_H
#include "haptic_nv.h"

#ifdef AW8624_DRIVER
struct aw_haptic_dts_name aw8624_dts_name[] = {
	{"aw8624_vib_lk_f0_cali", AW_LK_F0_CALI},
	{"aw8624_vib_mode", AW_MODE},
	{"aw8624_vib_f0_pre", AW_F0_PRE},
	{"aw8624_vib_f0_cali_percen", AW_F0_CALI_PERCENT},
	{"aw8624_vib_cont_drv_lev", AW_CONT_DRV1_LVL},
	{"aw8624_vib_cont_drv_lvl_ov", AW_CONT_DRV2_LVL},
	{"aw8624_vib_cont_td", AW_CONT_TD},
	{"aw8624_vib_cont_zc_thr", AW_CONT_ZC_THR},
	{"aw8624_vib_cont_num_brk", AW_CONT_NUM_BRK},
	{"aw8624_vib_f0_coeff", AW_F0_COEFF},
	{"aw8624_vib_tset", AW_CONT_TSET},
	{"aw8624_vib_sw_brake", AW_SW_BRAKE},
	{"aw8624_vib_bemf_config", AW_BEMF_CONFIG},
	{"aw8624_vib_duration_time", AW_DURATION_TIME},
	{"aw8624_vib_trig_config", AW_TRIG_CONFIG},
	{"aw8624_vib_brake_cont_config", AW_BRAKE_CONT_CONFIG},
	{"aw8624_vib_f0_trace_parameter", AW_F0_TRACE_PARAM},
};
extern struct aw_haptic_dts_name aw8624_dts_name[];
#endif

#ifdef AW8622X_DRIVER
struct aw_haptic_dts_name aw8622x_dts_name[] = {
	{"aw8622x_vib_lk_f0_cali", AW_LK_F0_CALI},
	{"aw8622x_vib_mode", AW_MODE},
	{"aw8622x_vib_f0_pre", AW_F0_PRE},
	{"aw8622x_vib_f0_cali_percen", AW_F0_CALI_PERCENT},
	{"aw8622x_vib_cont_drv1_lvl", AW_CONT_DRV1_LVL},
	{"aw8622x_vib_lra_vrms", AW_LRA_VRMS},
	{"aw8622x_vib_cont_drv1_time", AW_CONT_DRV1_TIME},
	{"aw8622x_vib_cont_drv2_time", AW_CONT_DRV2_TIME},
	{"aw8622x_vib_cont_wait_num", AW_CONT_WAIT_NUM},
	{"aw8622x_vib_cont_brk_gain", AW_CONT_BRK_GAIN},
	{"aw8622x_vib_cont_tset", AW_CONT_TSET},
	{"aw8622x_vib_cont_bemf_set", AW_CONT_BEMF_SET},
	{"aw8622x_vib_d2s_gain", AW_D2S_GAIN},
	{"aw8622x_vib_cont_brk_time", AW_CONT_BRK_TIME},
	{"aw8622x_vib_cont_track_margin", AW_CONT_TRACK_MARGIN},
	{"aw8622x_vib_sine_array", AW_SINE_ARRAY},
	{"aw8622x_vib_trig_config", AW_TRIG_CONFIG},
	{"aw8622x_vib_duration_time", AW_DURATION_TIME},
	{"aw8622x_vib_is_enabled_auto_brk", AW_ENABLE_AUTO_BRK},
	{"aw8622x_vib_is_enabled_track_en", AW_ENABLE_TRACK_EN},
};
extern struct aw_haptic_dts_name aw8622x_dts_name[];
#endif

#ifdef AW86214_DRIVER
struct aw_haptic_dts_name aw86214_dts_name[] = {
	{"aw86214_vib_lk_f0_cali", AW_LK_F0_CALI},
	{"aw86214_vib_mode", AW_MODE},
	{"aw86214_vib_f0_ref", AW_F0_PRE},
	{"aw86214_vib_f0_cali_percent", AW_F0_CALI_PERCENT},
	{"aw86214_vib_cont_drv1_lvl", AW_CONT_DRV1_LVL},
	{"aw86214_vib_lra_vrms", AW_LRA_VRMS},
	{"aw86214_vib_cont_drv1_time", AW_CONT_DRV1_TIME},
	{"aw86214_vib_cont_drv2_time", AW_CONT_DRV2_TIME},
	{"aw86214_vib_cont_brk_time", AW_CONT_BRK_TIME},
	{"aw86214_vib_cont_wait_num", AW_CONT_WAIT_NUM},
	{"aw86214_vib_cont_tset", AW_CONT_TSET},
	{"aw86214_vib_cont_bemf_set", AW_CONT_BEMF_SET},
	{"aw86214_vib_d2s_gain", AW_D2S_GAIN},
	{"aw86214_vib_cont_track_margin", AW_CONT_TRACK_MARGIN},
	{"aw86214_vib_sine_array", AW_SINE_ARRAY},
	{"aw86214_vib_duration_time", AW_DURATION_TIME},
	{"aw86214_vib_is_enabled_track_en", AW_ENABLE_TRACK_EN},
};
extern struct aw_haptic_dts_name aw86214_dts_name[];
#endif

#ifdef AW8623X_DRIVER
struct aw_haptic_dts_name aw8623x_dts_name[] = {
	{"aw8623x_vib_lk_f0_cali", AW_LK_F0_CALI},
	{"aw8623x_vib_mode", AW_MODE},
	{"aw8623x_vib_f0_pre", AW_F0_PRE},
	{"aw8623x_vib_f0_cali_percen", AW_F0_CALI_PERCENT},
	{"aw8623x_vib_cont_drv1_lvl", AW_CONT_DRV1_LVL},
	{"aw8623x_vib_lra_vrms", AW_LRA_VRMS},
	{"aw8623x_vib_cont_drv1_time", AW_CONT_DRV1_TIME},
	{"aw8623x_vib_cont_drv2_time", AW_CONT_DRV2_TIME},
	{"aw8623x_vib_cont_wait_num", AW_CONT_WAIT_NUM},
	{"aw8623x_vib_cont_brk_gain", AW_CONT_BRK_GAIN},
	{"aw8623x_vib_cont_tset", AW_CONT_TSET},
	{"aw8623x_vib_cont_bemf_set", AW_CONT_BEMF_SET},
	{"aw8623x_vib_d2s_gain", AW_D2S_GAIN},
	{"aw8623x_vib_cont_brk_time", AW_CONT_BRK_TIME},
	{"aw8623x_vib_cont_track_margin", AW_CONT_TRACK_MARGIN},
	{"aw8623x_vib_trig_config", AW_TRIG_CONFIG},
	{"aw8623x_vib_duration_time", AW_DURATION_TIME},
	{"aw8623x_vib_is_enabled_auto_brk", AW_ENABLE_AUTO_BRK},
	{"aw8623x_vib_is_enabled_track_en", AW_ENABLE_TRACK_EN},
};
extern struct aw_haptic_dts_name aw8623x_dts_name[];
#endif
#endif