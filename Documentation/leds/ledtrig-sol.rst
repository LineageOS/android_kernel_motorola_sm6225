================================
Sign-of-life LED trigger
================================

Description
===========

  This is a Sign-of-life LED trigger for QCOM Tri-led Driver.
      Refer -> kernel_platform/qcom/proprietary/devicetree/bindings/leds/leds-qti-tri-led.txt
  This trigger will register with LED core framework to individually
  get activated or deactivated.
  The trigger will be registered with the power_supply_class to get charging/discharging
  state and battery capacity.

Handle power-supply states for Charging/Discharging
===================================================

Set blink/solid Led settings as below

LED Brighness depend on Device Charging State
----------------------------------------------
  Full while Device Charging
  Half Brighness while not Charging

Led Functional Mode Depend on the Battery capacity
--------------------------------------------------
  Battery Level < LOW_CAP_THRESHOLD_DEFAULT
  Fast Blink "charging_low" led node(s)

LOW_CAP_THRESHOLD_DEFAULT < Battery Level < BATT_CAP_THRESHOLD_HIGH
  Slow Blink "charging" led node(s) (If device charging)
  Enable Solid 50% brightness for "charging" led node(s) (If device not charging)

BATT_CAP_THRESHOLD_HIGH < Battery Level < BATT_CAP_THRESHOLD_FULL
  Slow Blink "charging_full" led node(s) (If device charging)
  Enable Solid 50% brightness for "charging_full" led node(s) (If device not charging)

Battery Level ==  BATT_CAP_THRESHOLD_FULL
  Enable Solid 100% brightness for "charging_full" led node(s)

Label each node with corresponding colour in device tree
======================================================

  Qcom Tri-Led will have settings for each Led (R,G,B).
  The trigger driver will control with blink or solid pattern via labels nodes like
  charging_low node: Charging/Discharging and Low battery state
  charging node: Charging/Discharging state with medium battery
  charging_full node: Charging/Discharging state while battery near to Full.

Example of device tree setting
=============================

::

&pm8550_rgb {
	red {
		status = "ok";
		label = "charging_low";
		linux,default-trigger = "sign-of-life";
	};

	green {
		status = "ok";
		label = "charging_full";
		linux,default-trigger = "sign-of-life";
	};

	blue {
		status = "ok";
		label = "charging";
		linux,default-trigger = "sign-of-life";
	};
};

List of Methods in Sol trigger driver
====================================
	static int sol_trig_activate(struct led_classdev *led_cdev)
	{
	    /* Set default sol data register with power_supply notifier */
	}

	static void sol_trig_deactivate(struct led_classdev *led_cdev)
	{
		/* Unregister with power_supply notifier and Free sol data */
	}

	static int sol_led_update_status(led_name, blink, brightness, sol_data)
	{
		/* API to set sol led colour, brightness etc based on battery status */
	}
