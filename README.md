# esp32-multisensor-firmware

## Versión español

El firmware gestiona de forma simultánea tres tareas independientes que interactúan con sensores específicos:
* Sensor Ultrasonico HC-RS04
* Módulo con sensor de gestos GY-9960-3.3 (con chip APDS-9960)
* Módulo con unidad inercial GY-521 (con Mpu 6050) 

La tarea que interactúa con el sensor ultrasónico detecta a una frecuencia regular la distancia a la que está de un objeto.
La tarea que interactúa con el sensor de gestos informa si se ha producido un gesto reconocido por el sensor y la que interactua con el sensor inercial informa de los cambios en la acceleración y orientación relativa en los tres ejes cartesianos.

El diseño del firmware fue estructurado para aislar los drivers de los sensores de las placas ESP32 y del IDF, promoviendo una arquitectura modular.
Para facilitar la interacción con las funcionalidades del IDF, se han desarrollado wrappers personalizados que encapsulan las operaciones de comunicación I2C, control de GPIO, gestión del tiempo y manejo de errores. Esta capa de abstracción simplifica el código de los drivers, promueve la reutilización de los mismos en otras plataformas.

Con respecto a la API del I2C del ESP-IDF hay un nuevo diseño (ver[1]) (en mi opinión uno mejor que el anterior) pero que por alguna razón no funciona como se supone. De todos modos se incluye el código del wrapper para usar el nuevo diseño de la API y también para usar la versión legacy.
El driver lecacy no puede cohexistir con el nuevo. El firmware está testeado y funcionando con la versión legacy.  

[1]https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html

Estructura de directorios:

esp32-multisensor-firmware
    ├── main : directorio con la aplicación principal
    │   ├── esp32-multisensor-firmware.c
    |── mcu_interfaces/  : directorio con los wrappers para el esp-idf
    |   |── include/ : headers
    │   |── src/     : código fuente
    |── sensors/: directorio con los drivers
    │    |── include/ : headers
    |    |── src/     : código fuente de los drivers
     
El código de los drivers que está en el directorio sensors está completamente desacoplado del esp-idf y
de la placa esp32 e incluso del sistema operativo FreeRtos. Las llamadas a las funciones de librerías del idf necesarias para lograr la comunicación con los sensores, el manejo del tiempo o los mensajes se error se hacen a través de las interfaces provistas en el directorio mcu_interfaces. Las headers de este directorio también son agnósticas de la sdk, de modo que para portar el código de los drivers a otro sistema alcanza con reimplementar los prototipos de las funciones declaradas en el headers. 

## English version

The firmware simultaneously manages three independent tasks that interact with specific sensors:

* Ultrasonic sensor HC-SR04
* Gesture sensor module GY-9960-3.3 (with APDS-9960 chip)
* Inertial measurement unit module GY-521 (with MPU6050)

The task that interacts with the ultrasonic sensor regularly detects the distance to an object.
The task that interacts with the gesture sensor reports whether a recognized gesture has occurred, and the task that interacts with the inertial sensor reports changes in acceleration and relative orientation along the three Cartesian axes.

The firmware design was structured to isolate sensor drivers from the ESP32 boards and the IDF,
promoting a modular architecture.
To facilitate interaction with IDF functionalities, custom wrappers were developed that encapsulate I2C communication, GPIO control, time management, and error handling operations. This abstraction layer simplifies the code and promotes the reuse of drivers developed on other platforms.

The legacy driver can't coexist with the new driver. Include i2c.h to use the legacy driver or the other two headers to use the new driver.


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