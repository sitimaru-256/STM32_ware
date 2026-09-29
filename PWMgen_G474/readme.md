## wire connections
PD4 and PD5 pin is set as pull_up, but too weak to drive. It is recommended that add external 1kΩ resistor like that graph shown below.
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
