# BlinkLED
created drivers to implement interrupts and GPIO pin setup.

The files were created for the stm32F411 Discovery board.

These were made in stm32CubeIDE and so I didn't include the 
automatically created files such as the startup file, syscalls, and such.

In this example, the interrupt occurs at pin PD0. The Led is powered through
PD12, which is toggle on each interrupt. The interrupt is toggled on the falling
edge which is applied using a button.
