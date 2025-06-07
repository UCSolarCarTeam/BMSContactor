# BMSContactor

## 🪛 Set Up

Before flashing each contactor, line 44 in ContactorsIOCFile/Core/Src/main.c needs to be changed to the corresponding contactor. The options are:

- COMMON
- MOTOR
- ARRAY
- LV
- CHARGE
## 🧠 Logic

Overview: the Master BMS sends the wanted states for all the contactors. The contactor will see if it's in the wanted state. If it's not, it will call the Gatekeeper function.

This function will delegate what to do to get the wanted state. To open/close contactors, the changeSwitch function will do that. It will also check if that state is actually acheived.

A BPS Error [the contactor needs to open but it doesn't] is noticed, we plan to add a way that calls the interrupt right away.

The interrupt is incharge of sending message. The interrupt is called every 100 milliseconds. The heartbeat will be sent using this interrupt every 1 second and the general status will be sent every 100 milliseconds.
