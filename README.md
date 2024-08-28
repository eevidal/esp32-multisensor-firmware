# esp32-multisensor-firware

El diseño del firmware fue estructurado para aislar los drivers de los sensores de las placas ESP32 y del IDF, promoviendo una arquitectura modular.
Para facilitar la interacción con las funcionalidades del IDF, se han desarrollado wrappers personalizados que encapsulan las operaciones de comunicación I2C, control de GPIO, gestión del tiempo y manejo de errores. Esta capa de abstracción simplifica el código, promueve la reutilización de los drivers desarrollados en otras plataformas.

Con respecto a la API del I2C del ESP-IDF hay un nuevo diseño [1] (en mi opinión un mejor que el anterior) pero que por alguna razón no funciona como se supone. De todos modos se incluye el código del wrapper para usar el nuevo diseño de la API y también para usar la versión legacy. EL firmware está testeado con la versión legacy.  

[1]https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html

Estructura de directorios:

esp32-multisensor-firmware
    ├── main : folder with main application
    │   ├── esp32-multisensor-firmware.c
    |── mcu_interfaces/  : folder with idf wrappers
    |   |── include/ : folder with headers 
    │   |── src/     : folder source code of wrappers
    |── sensors/: folder with the sensors drivers
    │    |── include/ : folder with headers 
    |    |── src/     : folder source code of the drivers
    |──      

The firmware design was structured to isolate sensor drivers from the ESP32 boards and the IDF,
promoting a modular architecture.
To facilitate interaction with IDF functionalities, custom wrappers were developed that encapsulate I2C communication, GPIO control, time management, and error handling operations. This abstraction layer simplifies the code and promotes the reuse of drivers developed on other platforms.

The legacy driver can't coexist with the new driver. Include i2c.h to use the legacy driver or the other two headers to use the new driver.