# esp32-multisensor-

El diseño del firmware fue estructurado para aislar los drivers de los sensores de las placas ESP32 y del IDF, promoviendo una arquitectura modular.
Para facilitar la interacción con las funcionalidades del IDF, se han desarrollado wrappers personalizados que encapsulan las operaciones de comunicación I2C, control de GPIO, gestión del tiempo y manejo de errores. Esta capa de abstracción simplifica el código, promueve la reutilización de los drivers desarrollados en otras plataformas.


The firmware design was structured to isolate sensor drivers from the ESP32 boards and the IDF,
promoting a modular architecture.
To facilitate interaction with IDF functionalities, custom wrappers were developed that encapsulate I2C communication, GPIO control, time management, and error handling operations. This abstraction layer simplifies the code and promotes the reuse of drivers developed on other platforms.