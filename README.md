# STM32-BMM150
An STM32 code based on the Bosch BMM150 sensor API. The target hardware is the STM32 Black Pill (STM32F411 w/USB-C) connected to the BMM150 module via I2C. The code is a part of the Special Course in Space Technology being taught at the University of Turku.

## BMM150 sensor API

### Sensor overview

BMM150 is a standalone geomagnetic sensor for consumer market applications.
Performance and feature of BMM150 are carefully tuned and perfectly match the
demanding requirements of all 3-axis mobile applications such as electronic
compass, navigation or augmented reality.

### Target Application
- Magnetic heading information
- Tilt-compensated electronic compass
- Gyroscope calibration in 9-DoF application for mobile devices
- In-door navigation
- Gaming

### Feature
- Magnetic field data

### Important links

- [BMM150 product page](https://www.bosch-sensortec.com/products/motion-sensors/magnetometers-bmm150/)
- [BMM150 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf)
- [BMM150 shuttle board flyer](https://www.bosch-sensortec.com/media/boschsensortec/downloads/shuttle_board_flyer/bst-dhw-fl016.pdf)
- [Community support page](https://community.bosch-sensortec.com)

---
#### BMM150 sensor API copyright (C) 2020 Bosch Sensortec GmbH
#### UART IO retarget code copyright (C) Carmine Noviello
