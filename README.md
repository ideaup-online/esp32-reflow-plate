# Solder Reflow Plate Prototype

This is an ESP32 project for controlling a PCB-based solder reflow plate. The solder reflow plate used in this prototype was made using gerbers from [AfterEarthLTD's solder reflow plate](https://github.com/AfterEarthLTD/Solder-Reflow-Plate).

This project was created out of the need to assemble boards using solder reflow. I have some project ideas involving parts that have footprints that can't be soldered any other way.

I became aware of AfterEarthLTD's design after it was featured in a Great Scott! video on YouTube.

At the time I started this, ATmega328 chips are still not available and the code for the ATmega 4809 revision was not yet completed. So, I decided to send the gerbers for the ATmega 4809 revision to JLCPCB to be manufactured.

## Connecting the Solder Reflow Plate to Breadboard Prototype

The boards were populated with only the mosfet (with associated capacitor and zener diode) and the temp sensor (with associated capacitor). I also simply used a 1nF capacitor for both to reduce the number of parts I needed to purchase.

After that, an XT60 connector was soldered with pigtails to the pads for the barrel connector (I did not use the barrel connector). Leads were soldered for the positive side of the XT60, ground, 5V, the gate of the mosfet, and the output of the temp sensor. The positive side of the XT60 was picked up from the pad for the big capacitor that goes next to the barrel connector. Ground and 5V were picked up from the OLED connector through holes. The lead for the gate of the mosfet was soldered directly to the gate pin/pad of the mosfet (mosfet is installed on board). The temp sensor output was picked up from the ATmega 4809 footprint. The leads were terminated in a dupont connector for ease of breadboarding.

I powered all this from the XT60 using battery power of ~13.5V. That voltage was brought over to a breadboard where the rest of the components and the ESP32 were connected. 3.3V from the ESP32 is sent to the board via the 5V lead. This is fine because the only thing I populated that uses the 5V is the temp sensor and it is just as happy with 3.3V. The ground of the board is connected to the ground of the breadboard components. Finally, the gate and temp sensor voltage are passed to the breadboard via their leads.

The battery voltage is brought down to 5V with an XL4015 buck converter to power the ESP32 dev kit board. The ESP32 dev kit then provides 3.3V to everything else.

## Modifications to AfterEarthLTD's Design

Aside from the obvious of using an ESP32 instead of an ATmega, a couple of changes have been made.

The mosfet would not be turned on all the way by simply connecting the gate to one of the ESP32 GPIOs. The max output of 3.3V would not be sufficient. Therefore a DGD0215 gate driver has been used to drive the mosfet.

Also, the ESP32 ADC is not very linear. So, I opted to use a MAX11645 12-bit ADC and MCP1501-20 voltage reference. The LMT85 temp sensor tops out at around 1.9V on its output, so I chose the MCP1501-20 2.048V reference based on that.

Finally, an LED has been added to show when the heater is on (D1).

## 3D Printed Base

A simple 3D printed base has been created. This keeps the board from burning my desk when it gets hot. The STL file for it is here. It needs 4x M3x12 screws, 2x M3 washers, and 4x M3 nuts to assemble. I printed it in PETG on a Prusa Mini.

## Code

The code is not yet complete. The goal is to have the ability to have the board follow a solder reflow profile reasonably closely while being controlled and monitored via a web app running on the ESP32.

For now, there is a 128x64 OLED display just to monitor temperatures. It displays the value from the LMT85 as well as two K-type thermocouples attached via Adafruit MAX31855 breakout boards.

The current version of the code ensures that everything powers up without the heater coming on. The on-board button for GPIO0 can be used to turn on the heater (heater is only on while the button is pressed).

This readme will be updated as the code evolves.

## Should You Build One?

As of now, I would say no. My goal is to spin a new board based on the changes outlined above. I'd also like to try to make the working surface a bit larger than the current 50x70mm.

All that is going to take a little exploring and experimenting. That said, if you would like to do such exploration on your own, then by all means build this as is!
