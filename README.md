# esp32-multisensor-firmware


The firmware simultaneously manages three independent tasks that interact with specific sensors:

* Ultrasonic sensor HC-SR04
* Gesture sensor module GY-9960-3.3 (with APDS-9960 chip)
* Inertial measurement unit module GY-521 (with MPU6050)

The task that interacts with the ultrasonic sensor regularly detects the distance to an object.
The task that interacts with the gesture sensor reports whether a recognized gesture has occurred, and the task that interacts with the inertial sensor reports changes in acceleration and relative orientation along the three Cartesian axes.

The firmware design was structured to isolate sensor drivers from the ESP32 boards and the IDF,
promoting a modular architecture.
To facilitate interaction with IDF functionalities, custom wrappers were developed that encapsulate I2C communication, GPIO control, time management, and error handling operations. This abstraction layer simplifies the code and promotes the reuse of drivers developed on other platforms.

Regarding the ESP-IDF I2C API,

 there's a new design (see [1]) which I believe is an improvement over the previous one, but for some reason it's not working as expected. Nevertheless, the wrapper code is included to use both the new API design and the legacy version.

The legacy driver cannot coexist with the new one. The firmware has been tested and is working with the legacy version.

[1]https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html

### Folders structure:

    esp32-multisensor-firmware
        ├── main : folder with main application
        │   ├── esp32-multisensor-firmware.c
        |── mcu_interfaces/  : folder with idf wrappers
        |   |── include/ : folder with headers 
        │   |── src/     : folder source code of wrappers
        |── sensors/: folder with the sensors drivers
        │    |── include/ : folder with headers 
        |    |── src/     : folder source code of the drivers
     
The driver code located in the sensors directory is completely decoupled from the ESP-IDF, the ESP32 board, and even the FreeRTOS operating system. Calls to IDF library functions required for sensor communication, time management, or error handling are made through interfaces provided in the mcu_interfaces directory. The headers in this directory are also SDK-agnostic, meaning that to port the driver code to another system, it is sufficient to reimplement the prototypes of the functions declared in the headers of mcu_interfaces.

## IMU Disclaimer:
The MPU-6050 has the capability to connect to another device (usually a magnetometer) via I2C in master mode.To be complete, the driver should include the necessary functions to provide this capability, but due to time constraints, I didn't even consider them. The same goes for several functions that could be in a higher level of abstraction layer where a Kalman filter or similar is implemented to calculate the device's pose.
