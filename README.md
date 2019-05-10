# ledmushroom


![alt text](https://github.com/vireshsoedhwa/ledmushroom/blob/master/mushroom.jpg)

Small summer project I set up to experiment with FreeRTOS:

3d printed the mushroom (3d model from https://www.thingiverse.com/thing:930147)
https://www.thingiverse.com/thing:930147/attribution_card

Led controller: TLC5940
Microcontroller: stm32f103(blue pill)
IDE used: System workbench for STM32 with STMCubeMX to generate boilerplate code

Created a semaphore to cycle the RGB leds with different colors and cycles. 

FreeRTOS Task 1 : responsible for choosing the *random parameters that determines the rate of change and frequency for each RGB channel
*To achieve better random values I set one of the pins to analog and read the analog value to seed the rand() function. 

FreeRTOS Task 2 : applies the parametes to a sine function(amplitude and frequency)
Global interrupt: Set up an interrupt on a timer that updates the LED intensity values using SPI.

Result: with different phases between each channel a nice pattern of colors appear. 

For the TLC5940 I refactored the code from the following source to work with STM32
[Demystifying the TLC5940 A free book/library by Matthew T. Pandina](https://sites.google.com/site/artcfox/demystifying-the-tlc5940)


[Youtube video](https://youtu.be/zM3H_MbjTRc)
