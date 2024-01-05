/*****************************************************************************
* Copyright(c) O2Micro, 2014. All rights reserved.
*
* O2Micro battery gauge driver
* File: table.h
*
* $Source: /data/code/CVS
* $Revision: 4.00.01 $
*
* This program is free software and can be edistributed and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This Source Code Reference Design for O2MICRO Battery Gauge access (\u201cReference Design\u201d) 
* is sole for the use of PRODUCT INTEGRATION REFERENCE ONLY, and contains confidential 
* and privileged information of O2Micro International Limited. O2Micro shall have no 
* liability to any PARTY FOR THE RELIABILITY, SERVICEABILITY FOR THE RESULT OF PRODUCT 
* INTEGRATION, or results from: (i) any modification or attempted modification of the 
* Reference Design by any party, or (ii) the combination, operation or use of the 
* Reference Design with non-O2Micro Reference Design.
*
* Battery Manufacture: ATL
* Battery Model: M1892
* Absolute Max Capacity(mAhr): 3661
* Limited Charge Voltage(mV): 4400
* Cutoff Discharge Voltage(mV): 3200
* Equipment: "JY1-5","JY1-5","JY1-5","JY1-5","JY1-5","JY1-5","JY1-5","JY1-5","JY1-5","JY1-5","JY1-5","JY1-5","JY2-5","JY2-5","JY2-5","JY2-6","JY2-6","JY2-6","JY2-6","JY2-6","JY2-6","JY2-6","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-6","JY2-6","JY2-6","JY2-6","JY2-6","JY2-6","JY2-6","JY2-6","JY2-6","JY2-5","JY1-6","JY1-6","JY1-6","JY1-6","JY1-6","JY1-6","JY1-8","JY1-8","JY1-8","JY1-8","JY1-6","JY1-6","JY1-6","JY1-6","JY1-6","JY1-6","JY1-6","JY1-6","JY1-6","JY1-6","JY1-6","JY1-8","JY1-7","JY1-7","JY1-7","JY1-7","JY1-7","JY1-7","JY1-3","JY1-3","JY1-3","JY1-5","JY1-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY2-5","JY1-8","JY1-8","JY1-2","JY1-2","JY1-2","JY2-6","JY2-6","JY2-6"
* Tester: "SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ","SZ"
* Battery ID: "M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_1","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_2","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_3","M1892_4","M1892_4","M1892_4","M1892_2","M1892_2","M1892_2"
* Version = V00.8D.00
* Date = 2018.3.26
* Comment = 
*           
*****************************************************************************/

#ifndef _TABLE_H_
#define _TABLE_H_


#define OCV_DATA_NUM 	 65
#define CHARGE_DATA_NUM 		 65


#define XAxis 		 86
#define YAxis 		 10
#define ZAxis 		 9

#define BATTERY_ID_NUM 		 93


/****************************************************************************
* Struct section
*  add struct #define here if any
***************************************************************************/
typedef struct tag_one_latitude_data {
 	 int32_t 			 x;//
 	 int32_t 			 y;//
} one_latitude_data_t;


/****************************************************************************
* extern variable declaration section
***************************************************************************/
extern const char *battery_id[] ;

extern const char *table_version;

#endif

