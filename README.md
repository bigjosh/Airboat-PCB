Airboat
=======

Firmware and board files for Dame Products Airboat. More product info here...

http://www.dameproducts.com/

Note that this is a fork of the original EVA V2 design which is available here...

https://github.com/bigjosh/vibe-v2

Changes include....

1. Much smaller form factor.
2. MCU switched to smaller ATTINY25/45/85 device.
3. Battery charge controller switched to smaller MCP73832 device.


User Interface
--------------

###Transit Lockout Mode
Device ships in Transit Lockout Mode to prevent inadvertently turning on while in transit. In this mode, the button is completely disconnected and non-functional. To escape from Transit Lockout Mode into normal operating mode, you must connect the unit to the charger.

You can put the device back into Transit Lockout Mode by holding the button down for longer than 10 seconds. During the countdown, the green LED will briefly flash once per second.

After the countdown expires, the  green LED will fade out to indicate that Transit Lockout Mode has been activated.

Note: this mode also protects the unit from over discharging the battery in case the button gets stuck down because something is continuously pressing on it.

###Normal Operating Mode
Device is normally in a low-power off state.

A long button press (~0.25 sec) goes straight to off state from any speed. 

A short button press steps to the next speed setting upon button release. There are 3 speed settings (low, medium, high) and a 4th button press returns to off state.

When the battery gets low, the motor turns off and the red LED lights for about 1 second. Subsequent button pushes blink the red LED again to indicate that there is not enough power to turn on. Note that Lithium Polymer batteries recover some voltage while resting, so it is possible to briefly turn the unit back on even after it has automatically turned off from low battery. Doing this repeatedly will lead to shorter and shorter on times until eventually the battery is too depleted to have any recovery left.

Because the battery has reduced voltage under load, there are different cutoff voltages for initial turn on and continuing operation. The battery must be at least 3.3 volts for the motor to start, but once started it will continue to run until the battery drops to 3.1 volts. 

For user feedback, both LEDs light for about 250ms when the button is pressed. If the button is held down, it the  LEDs will flash for 100ms every second until either the button is released or <a href="#transit-lockout-mode">Transit Lockout Mode</a> is activated.  

The green LED also indicates charger status: 

* Off means no charger connected
* On solid, full brightness means charger is connected and battery is fully recharged (LED blinks for 62ms once every 8 seconds)
* Slow pulse (1Hz) means charging


Motor is always off while charger connected. Button state is ignored while the charger is connected.

Test Mode
---------
On initial power-up, the devices enters a test mode...

1. You should see the LEDs alternate blinking red and green at about 10Hz. This lets you visually verify both LEDs are working. 
  1. If you don't see anything on power-up, then either both LEDs are broken or there is some worse problem.
  2. If you see a single LED blinking on and off at 10Hz, then the other LED is bad.
  3. If you see the while LED on at 50%, then you probably have a stuck button. 

2. Press the button. The green LED should light solid at 50% brightness for as long as you hold down the button. The green LED should go out when you release the button.
  1. If the LEDs continue to alternate flash after you push the button, then you probably have a bad button or bad connection from the board to the button. 
  2. If the green LED stays lit at 50% brightness after you release the button, then you probably have a stuck button.

Note that if the button is pressed upon initial power-up, then the normal test mode red/green blinking is skipped (because this is typically terminated by a button press).

Note that both above LED indications time out after about 30 seconds to avoid killing the battery. If the button stays down past the timeout, the the unit will eventually enter <a href="#transit-lockout-mode"Transit Lockout Mode</a>. 

Note that test mode only happens on initial power up. The board is very low power so it can continue to operate off the residual charge in the capacitor for a long time (days-months) after the battery is disconnected. Because of this, you must follow this procedure to re-enter test mode once the device has been powered up (it is not enough just to remove the battery and replace it)... 

1. disconnect all power (unplug the battery, unplug charger)
2. push and hold the button for a second to exhaust all the residual power from the capacitors
3. reconnect power and test mode should start again
 

Features
--------
* Current draw of <0.1uA when idle, so battery self-drain is likely the limiting factor for maximum off time.
* Zero latency button debounce.
* Motor output power is scaled to battery voltage so motor speed stays constant though battery discharge (at least until there is not enough voltage left to maintain speed). 
* Code size of about 1.5K easily fits into cheap parts. 
* A full processor WatchDog reset is executed every time the motor is turned off or the charger is unplugged. This hopefully makes the unit more robust to failures.

Connections
-----------
The specified battery is a 160mAh Polymer Li-ion. Positive battery wire connects to pad closest to the LED/jack end  of the board.

The specified motor is a vibration type nominally rated for 130mA at 3V.
