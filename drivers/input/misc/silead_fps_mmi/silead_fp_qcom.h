/*
 * @file   silead_fp_qcom.h
 * @brief  Contains silead_fp Qualcomm platform specific head file.
 *
 *
 * Copyright 2016-2021 Gigadevice/Silead Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * ------------------- Revision History ------------------------------
 * <author>    <date>   <version>     <desc>
 * Bill Yu    2018/5/2    0.1.0      Init version
 *
 */

#ifndef __SILEAD_FP_QCOM_H__
#define __SILEAD_FP_QCOM_H__

#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>

struct fp_plat_t {
    u32 qup_id;
#ifdef QSEE_V4
    u32 max_speed_hz;
#else
    /* pinctrl info */
    struct pinctrl  *pinctrl;
    struct pinctrl_state  *active;
    struct pinctrl_state  *sleep;
#ifdef BSP_SIL_POWER_SUPPLY_PINCTRL
    struct pinctrl_state *pins_avdd_h, *pins_vddio_h;
#endif /* BSP_SIL_POWER_SUPPLY_PINCTRL */
    /* clock info */
    struct clk    *core_clk;
    struct clk    *iface_clk;
#endif /* QSEE_V4 */
};

#endif /* __SILEAD_FP_QCOM_H__ */

/* End of file silead_fp_qcom.h */
