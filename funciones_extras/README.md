# FUNCIONES EXTRAS
```
├── funciones_extras
    │
    ├── launch
    │     └── nodo_de_movimiento.launch.py: Lanza un nodo de la clase 
    │
    ├── src
    │     ├── loggeo_de_datos.cpp: descripción del nodo para almacenamiento de datos relevantes en      │     │                         archivos txt.
    │     ├── trayectorias_predefinidas.cpp: archivo que ejecuta trayectorias definidas por el usuario  │     │                                   en moveit.
    │     └── nodo_de_comunicación.cpp: nodo para verificación de comunicación usb.
    │
    ├── README.md
    ├── CMakeLists.txt
    ├── sbot_hi.xml: informacion para ros2 sobre la libreria en cuestión.
    └── package.xml

Para lograr que las funciones de comunicación puedan ser ejecutadas de manera correcta es necesario verificar el nombre de los puertos conectados con los conversores usb-uart, para ello se utiliza el comando PYTHON3 -M serial.tools.list_ports()

Además es necesario activar los permisos de escritura en dichos puertos y agregar al usuario al grupo que tiene permisos de escritura y lectura sobre los mismos. De ser necesario también se debe dotar al ejecutable de dichos permisos.
