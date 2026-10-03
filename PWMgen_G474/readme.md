## wire connections
The PC4 and PC5 pins are configured as pull-ups, but their drive strength is too weak. Adding an external 1 kΩ resistor is recommended, as shown in the figure below.
### connecting around the MCU
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
### toggle switch(example)
```
            /———————\       =====[===]
ACC—————————|       |  ========
GND—————————|       ======
BRK—————————|       |
            \———————/
```
## PWM output
phase U: PC0
phase V: PC1
phase W: PC2
