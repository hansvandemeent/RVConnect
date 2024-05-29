# RVConnect

### Making a Smart (Mobile) Home

The goal is to connect a recreational vehicle (RV) to Smart Home via an LTE or LoRa portal.
This is to obtain information about location, movement, battery status and surroundings.

**Note: This is not a certified SCM Track and Trace system as required by insurance.**

**Note: Work in progress, basic functions working.**

![screenshot](docs/png/RVConnect%20map.png)

The * *Location* * data provides a Track and Trace of the RV. 

* *Movement* * reacts on small movements of the RV, caused by driving or a collision.

A RV has two or more * *Batteries* *. The car battery and one or more household batteries.
During winter most RV’s are stored for a long time without charging batteries. The level of these batteries is reported once an hour.

* *Environment* * values like temperature and humidity can cause severe damage to the RV. Freezing can damage the water tanks and humidity the construction of the RV. This is specially the case during the winter storage.
Both values are updated every hour. 

In the near future other systems like heating, lights, security and * *?* * become smart. 

The nature of an RV is traveling so we need coverage in a great part of Europe. So we need data roaming.

RV Connect must be able to operate unattended for a long period (6 months).

Power is supplied by the car or household battery. Both have a large capacity (up to 90 Ah)
The app will be in Deep Sleep during most of the time to save energy consumption.
It will wake up at fixed times or with a movement.

User actions are not executed immediately but as soon as the app wakes up from Deep Sleep.

If tests show that the app can be continuously online (energy and data usage), then this is an option.

The portal used is [KPN Things platform](https://docs.kpnthings.com/dm/), a LTE portal using SenML data format.

Sensor data is transferred via a webhook to Home Assistant.
Automations in Home Assistant check for temperature and humidity limits or movements and notify user(s) when needed.
The location of the RV is tracked for the past 24 hours.


### LoRa or LTE
The coverage of LoRa TheThingsNetwork is still too low. Using KPN LoRa can solve this but is only available in The Netherlands.
LTE has a full coverage in the Netherlands and a large part of Western Europe.

### The * *RVConnect* * application
Developed in C++ on PlatformIO.
Debugging can be done with a Segger J-Link, but most of the time I use my trace software.

### Hardware
Hardware is a Sodaq SARA development board. However this board is no longer available.
Plan is to redo this project with Nordic Thingy:91

### * *IoT Portal* *
First test was with HologramIO. However HologramIO had a large overhead of management data (700 bytes on a 50 bytes message). Advice was to collect multiple measurements and send them in 1 package. But that kills the tracking capability of the application.

Choice is now [KPN Things platform](https://docs.kpnthings.com/dm/).
Data from the RVConnect app is forwarded to the Home Assistant webhook.
A copy is sent to [webhook.site](https://webhook.site) for testing.

### * *Home Assistant* * Smart Home
I use [Home Assistant](https://www.home-assistant.io) already for a long time at home. It handles my lights, heating, solar panels, screens, air quality, alarm system, multi-media.
So it was a logical choise for my mobile home.

![screenshot](docs/png/HA.png)

Temperature, humidity and location checks are performed in Home Assistant. 
These have not yet been implemented in the current version.
* *Notificationa* * are sent when the temperature or humidity is too high or low.
This also happens when the vehicle is moving or due to a collision


