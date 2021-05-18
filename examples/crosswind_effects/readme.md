# Euler roll/yaw problems with crosswind

## rolling loop OK with no crosswind
### 3D view of maneuver showing level entry, rolling right through a total of 360 degrees. Color code shows upright and level as green, knife-edge is yellow and inverted is blue. Roll tolerance is 10 degrees; anything more than 19 degrees from upright, inverted or knife-edge is coded red.

![zero crosswind loop](w0_1_3Da.svg)

### Euler roll and pitch (eroll/epitch) are identical to maneuver roll and pitch (roll/pitch)

![zero crosswind loop](w0_1_eulerVmp.svg)

### 3D view of rolling loop with crosswind of 5 m/sec from the South (wind velocity (5,0,0) in ENU frame). View from above showing yaw (to South) required for wind correction.

![5 m/sec crosswind loop](w5N_1_3Da.svg)

## Euler roll and pitch differ from "manevuer plane" values when crosswind is present

### but one can calculate the "correct" angles given the correct maneuver heading. Note that the maneuver heading will be either along or across the aerobatic box when on vertical lines; otherwise ground course defines the maneuver heading.

![5 m/sec crosswind loop](w5N_1_eulerVmp.svg)
