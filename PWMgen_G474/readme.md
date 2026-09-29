## wire connections
The PD4 and PD5 pins are configured as pull-ups, but their drive strength is too weak. Adding an external 1 kΩ resistor is recommended, as shown in the figure below.
```
                  /———————————\
3V3——————————+————|3V3        |
             |    |           |
         +———+    |           |
         |   |    | STM32G474 |
        [R] [R]   |           |
         |   |    |           |
BRK——————+———)————|PC5        |
             |    |           |
ACC——————————+————|PC4        |
                  |           |
GND———————————————|GND        |
                  \___________/
```
## PWM output
phase U: PC0
phase V: PC1
phase W: PC2
