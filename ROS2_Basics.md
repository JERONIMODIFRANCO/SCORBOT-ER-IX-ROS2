# ROS2 
ROS 2 (Robot Operating System 2) es un marco de software para el desarrollo de robótica, proporcionando una plataforma sólida para crear sistemas robóticos complejos. Es un framework que se basa en un sistema operativo, permitiendo abstractizar el hardware del software, lo que facilita el desarrollo de programas para robots sin necesidad de contar con hardware físico en la escena. Algunas características clave de ROS 2 incluyen:
Arquitectura de nodos: ROS 2 se basa en la idea de nodos, que son procesos independientes que se ejecutan y se comunican entre sí mediante mensajes.
Herramientas y paquetes adicionales: ROS 2 incluye una serie de herramientas y paquetes adicionales para facilitar el desarrollo de robótica, como mapas y localización (2D/3D SLAM), navegación, percepción, representación de marcos de coordenadas y simulación.
Compatibilidad con ROS 1: Aunque ROS 2 es una mejora significativa en comparación con ROS 1, aún es compatible con algunos componentes de ROS 1 y permite la colaboración con sistemas híbridos que utilizan ambos.

# ¿Qué es un paquete?
ROS2 utiliza **paquetes** para organizar sus programas. Se puede pensar en un paquete como **todos los archivos que contiene un programa ROS2 específico**; todos sus archivos C++, archivos de Python, archivos de configuración, archivos de compilación, archivos de lanzamiento y archivos de parámetros.
Se puede crear dos tipos de paquetes: paquetes de Python y paquetes de CMake (C++).

Los paquetes de Python contendrán ejecutables de Python. Cada paquete de Python tendrá la siguiente estructura de archivos y carpetas:
package.xml - Archivo que contiene metainformación sobre el paquete (mantenedor del paquete, dependencias, etc.).
setup.py - Archivo que contiene instrucciones sobre cómo compilar el paquete.
setup.cfg - Archivo que define dónde se instalarán los scripts.
/<nombre_del_paquete> - Este directorio siempre tendrá el mismo nombre que su paquete. Colocarás todos tus scripts de Python dentro de esta carpeta. Ya contiene un archivo __init__.py vacío de forma predeterminada.
Algunos paquetes pueden contener carpetas adicionales. Por ejemplo, la carpeta launch contiene los archivos de lanzamiento del paquete.

La estructura de un proyecto CMake consta de los siguientes elementos:
Carpeta launch: Contiene archivos de lanzamiento.
Carpeta src: Contiene archivos de código (CPP, Python).
Archivo CMakeLists.txt: Lista de reglas de CMake para la compilación.
Archivo package.xml: Metadatos del paquete y dependencias.

Los paquetes se organizan dentro de espacios de trabajo en ROS 2

```Paquetes python
ros2_ws/
    src/
        my_package/
            package.xml
            setup.py
            ...
        my_package_2/
            package.xml
            setup.py
            ...
        my_package_x/
            package.xml
            setup.py
            ...
```

```Paquetes Cmake
ros2_ws/
    src/
        my_package/
            package.xml
            CMakeLists.txt
            ...
        my_package_2/
            package.xml
            CMakeLists.txt
            ...
        my_package_x/
            package.xml
            CMakeLists.txt
            ...
```

 **'ros2'** se utiliza para todos los comandos ROS2. 
 

# Crear un paquete
El espacio de trabajo de ROS2 es el directorio en tu disco duro donde residen tus paquetes de ROS2 para que sean utilizados por ROS2. 
El directorio src es una convención en ROS 2, el cual contiene todos los paquetes creados. Cada vez que desees crear un paquete, debes estar en este directorio nombre_del_paquete/src
Para crear un paquete en ROS 2, primero debes asegurarte de estar en el directorio src de tu espacio de trabajo. Luego, puedes utilizar el siguiente comando para crear un nuevo paquete:
Para un paquete de CMake:
```bash
ros2 pkg create --build-type ament_cmake <nombre_del_paquete> dependencies <package_dependencies>
```
Para un paquete de Python:
```bash
ros2 pkg create --build-type ament_python <nombre_del_paquete> dependencies <package_dependencies>
```
El nombre del paquete que deseas crear se especifica con <package_name>, y las dependencias del paquete se definen con <package_dependencies>
Es una buena idea construir tu paquete después de haberlo creado. Es la forma más rápida de determinar si las dependencias que has listado pueden resolverse y verificar que no hay errores en los datos ingresados
```
cd ~/nombre_del_paquete/
colcon build
```
```shell
source install/setup.bash
```

Compilar un paquete
```shell
colcon build
```
Para compilar un proyecto CMake, es necesario compilar cada vez que se cambie cualquier archivo, incluso archivos de Python o de lanzamiento que no necesiten compilación. Además, debido a que colcon build realiza una instalación y utiliza los archivos en ~/ros2_ws/install, los archivos ejecutados, es necesario ejecutar colcon build cada vez.
Es importante realizar este paso en la direccion del paquete y no en el src del mismo.
```shell
cd ~/ros2_ws
colcon build
```
Después de compilar un paquete, es necesario ejecutar el comando source para actualizar el entorno de trabajo y asegurar que las últimas modificaciones o actualizaciones sean tomadas en cuenta.
```shell
source install/setup.bash
```
Para compilar solo los paquetes en los que has realizado cambios en ROS 2, puedes utilizar el siguiente comando:
```cmd
colcon build --packages-select <nombre_del_paquete>
```

 # Que es un Launch File?
 Los archivos de lanzamiento son útiles porque:
Facilitan el inicio de múltiples nodos: Permiten iniciar y configurar múltiples ejecutables que contienen nodos de ROS 2 simultáneamente con un solo comando.
Permiten la configuración de los nodos: Facilitan la configuración de los nodos, por ejemplo, configurando parámetros o reemplazando valores en el archivo de lanzamiento durante la ejecución.
Proporcionan flexibilidad: Los archivos de lanzamiento pueden ser escritos en Python, XML o YAML, lo que permite elegir el formato que mejor se adapte a las necesidades específicas de tu proyecto.
Facilitan la organización y el desarrollo: Permiten estructurar archivos de lanzamiento para proyectos grandes, lo que facilita su reutilización en diferentes aplicaciones y contextos.

# Modifying the setup.py File
El archivo setup.py contiene todas las instrucciones necesarias para compilar correctamente tu paquete. 
El principal objetivo de este código es generar un ejecutable a partir del script que creaste hace unos momentos. Para lograrlo, trabajas con un diccionario llamado entry_points. Dentro de este, encuentras un array llamado console_scripts. Este array se utiliza para definir los comandos de consola que se generarán a partir de las funciones o scripts de tu paquete. Al utilizar entry_points con console_scripts, puedes especificar los comandos de consola que deseas que estén disponibles una vez que tu paquete esté instalado. Estos comandos de consola se generarán automáticamente y estarán disponibles para su uso.
```python 
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'

setup(
    
    #code
    ...
    #code
    
    entry_points={
            'console_scripts': [
                'simple_node = my_package.simple:main'
            ],
        },
    
    #code
    ...
    
)
```
Con estas líneas, añades un punto de entrada a tu script anterior, simple.py. Por ejemplo, puedes ver esta línea de la siguiente manera:
python
'<executable_name> = <package_name>.<script_name>:main'

En este caso, estás generando un nuevo nodo ejecutable llamado simple_node. Este ejecutable se genera utilizando un script llamado simple.py dentro de un paquete llamado my_package.
Además, hay otras cosas que puedes definir en un archivo setup.py. Para que colcon encuentre los archivos de lanzamiento durante el proceso de compilación, es necesario informar a las herramientas de configuración de Python sobre tus archivos de lanzamiento utilizando el parámetro data_files del archivo setup.py.
```python 
import os
from glob import glob
from setuptools import setup

package_name = 'my_package'

setup(
    
    #code
    ...
    #code
    
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name), glob('launch/*.launch.py'))
        ],
    
    #code
    ...
    #code
    
)
```
El objetivo de este código es instalar los archivos de lanzamiento. Por ejemplo, con el paquete llamado my_package, esto instalará todos los archivos de lanzamiento de la carpeta launch/ en ~/ros2_ws/install/my_package/share/my_package/.


Modifica el archivo CMakeLists.txt para generar un ejecutable a partir del archivo C++ que has creado.
Esto es algo que se requiere al trabajar en ROS2 con C++.
Cuando se programa en C++, es necesario crear binarios (ejecutables) de tus programas para ejecutarlos. Para ello, deberás modificar el archivo CMakeLists.txt de tu paquete para indicar que deseas crear un ejecutable de tu archivo C++
```cmake
add_executable(simple_node src/simple.cpp)
ament_target_dependencies(simple_node rclcpp)

install(TARGETS
   simple_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```

```cmake
add_executable(simple_node src/simple.cpp)
```
Esta línea genera un ejecutable a partir del archivo simple.cpp, que se encuentra en la carpeta src de tu paquete. Este ejecutable se llamará simple_node.
```cmake
ament_target_dependencies(simple_node rclcpp)
```
Esta línea agrega todas las dependencias del objetivo ament del ejecutable.
```cmake
install(TARGETS
   simple_node
   DESTINATION lib/${PROJECT_NAME}
 )
```
Este fragmento instalará nuestro nodo ejecutable (simple_node) en nuestro espacio de instalación dentro del espacio de trabajo de ROS2. Por lo tanto, este ejecutable se colocará en el directorio del paquete de tu espacio de instalación, que se encuentra, de forma predeterminada, en ~/ros2_ws/install/my_package/lib/my_package/.
```cmake
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```
El objetivo de este código es instalar los archivos de lanzamiento. Por ejemplo, con el paquete llamado my_package, esto instalará todos los archivos de lanzamiento de la carpeta launch/ en ~/ros2_ws/install/my_package/share/my_package/launch.

Por lo tanto, para lanzar programas, tendrás dos opciones:

- Inicie el programa ROS2 ejecutando directamente el **archivo ejecutable**.

- Inicie el programa ROS2 iniciando **launch file**.
```
ros2 run <package_name> <executable_file>
```
```
ros2 launch <package_name> <launch_file>
```

# NODOS 
En ROS 2, cada nodo debe ser responsable de un módulo único (por ejemplo, un nodo para controlar los motores de las ruedas, otro para controlar un sensor láser, etc.). Cada nodo puede comunicarse con otros nodos a través de diferentes métodos.
![image](https://github.com/JERONIMODIFRANCO/SCORBOT-ER-IX-ROS2/assets/95137387/940082c3-d29b-46c8-8937-b60ec247cefd)
En ROS 2, cada nodo esencialmente representa un programa de ROS. Para listar todos los nodos en ejecución en un sistema, se utiliza el comando ros2 node list. Este comando muestra los nombres de todos los nodos en ejecución, lo que resulta especialmente útil al interactuar con un nodo o al supervisar un sistema con múltiples nodos.

# Bibliotecas de cliente
Las bibliotecas de cliente de ROS permiten que los nodos escritos en diferentes lenguajes de programación se comuniquen entre sí. La biblioteca de cliente de ROS básica (RCL) implementa la funcionalidad estándar necesaria por varias APIs de ROS. Esto facilita la creación de bibliotecas de cliente específicas para cada lenguaje de programación.
El equipo de ROS 2 mantiene actualmente las siguientes bibliotecas de cliente:
rclcpp: Biblioteca de cliente en C++.
rclpy: Biblioteca de cliente en Python.

# TOPICOS
```shell
 ros2 topic -h
 ros2 topic list
```
Para publicar algo en un topico en ROS 2, primero debes verificar qué tipo de interfaz utiliza el tema
```shell
ros2 interface list
```
Un suscriptor es un nodo que lee información de un tema.
Las interfaces en ROS 2 se dividen en los siguientes grupos:
Mensajes: Se encuentran en archivos .msg. Son archivos de texto simples que describen los campos de un mensaje de ROS. Se utilizan para generar código fuente para mensajes en diferentes lenguajes de programación
Servicios: Se encuentran en archivos .srv. Están compuestos por dos partes: una solicitud y una respuesta. Ambas son declaraciones de mensaje
Acciones: Se encuentran en archivos .action. Están compuestos por tres partes: un objetivo, un resultado y un feedback. Cada parte contiene una declaración de mensaje
Estos grupos de interfaces proporcionan una forma estructurada de definir la comunicación entre los nodos en un sistema de ROS 2, lo que facilita el intercambio de datos y la interacción entre los componentes del sistema
El comando ros2 interface show se utiliza para obtener información sobre una interfaz específica en ROS 2
puedes enviar mensajes a un tema (publicar en un tema) y leer información de un tema (suscribirse a un tema) utilizando el comando ros2 topic
Para publicar en un tema, puedes utilizar el comando ros2 topic pub <nombre_del_tema> <tipo_de_mensaje> "<datos>"
Para leer información de un tema, puedes utilizar el comando ros2 topic echo <nombre_del_tema>
Imagina un tema como un conducto a través del cual fluye la información, un canal donde los nodos pueden leer o escribir información. 
Las configuraciones de calidad de servicio (QoS) de un nodo de publicador y un nodo de suscriptor deben ser compatibles.

# Servicios
En ROS 2, los servicios se utilizan para comunicaciones de tipo "llamada-respuesta". A diferencia de los temas, donde la información se publica y se suscribe de forma continua, un servicio solo proporciona datos cuando es llamado explícitamente por un cliente
Puedes tener múltiples clientes que utilicen el mismo servidor de servicio en ROS 2, pero solo puedes tener un servidor para un servicio específico.
![image](https://github.com/JERONIMODIFRANCO/SCORBOT-ER-IX-ROS2/assets/95137387/40e66b27-c67e-44b9-8483-58d4a4c1bc93)
Comandos basicos: 
```
ros2 service list
```
El comando ros2 service call se utiliza para llamar a un servicio en ROS 2 y enviar una solicitud. 
```shell
ros2 service call <service_name> <service_type> <value>
```
<mark style="background: #FF5582A6;">This is VERY IMPORTANT in Service types because it separates the **request** part of the message from the **response**</mark>. It is structured like this:
```xml
<request>
---
<response>
```
En ROS 2, un cliente de servicio puede realizar dos tipos de llamadas a un servicio:
Asíncronas: Las llamadas asíncronas son seguras y se recomiendan ampliamente para evitar bloqueos en otros procesos ROS y no ROS. Un cliente de servicio asíncrono realiza la solicitud y devuelve inmediatamente un "future", que indica si la solicitud y la respuesta están completas. El "future" puede ser consultado en cualquier momento para obtener la respuesta
Síncronas: Las llamadas sincrónicas bloquean la ejecución del nodo hasta que se complete la solicitud y la respuesta. Estas solicitudes pueden ser peligrosas, ya que pueden causar bloqueos innecesarios y problemas de sincronización. En ROS 2, se recomienda utilizar solicitudes asíncronas en lugar de sincrónicas, a menos que sea necesario

# Acciones 
En ROS 2, las acciones son muy similares a los servicios. Cuando se llama a una acción, se está llamando a una funcionalidad que otro nodo está proporcionando. Además, las acciones se basan en un modelo cliente-servidor, al igual que los servicios.
Sin embargo, hay dos diferencias principales entre las acciones y los servicios:
Las acciones son preemptibles. Esto significa que se puede cancelar una acción mientras se está ejecutando.
Las acciones proporcionan feedback. Esto significa que, mientras se está ejecutando la acción, el servidor puede enviar feedback al cliente.
![image](https://github.com/JERONIMODIFRANCO/SCORBOT-ER-IX-ROS2/assets/95137387/a452b096-21fc-42bc-adcd-d71861165189)
Además, las acciones utilizan servicios para manejar el objetivo y el resultado, y utilizan temas para manejar el feedback
![image](https://github.com/JERONIMODIFRANCO/SCORBOT-ER-IX-ROS2/assets/95137387/c74ee907-6949-4257-ad41-a6278b170a33)
En resumen, el flujo de trabajo de las acciones en ROS 2 es el siguiente:
El cliente envía un objetivo al servidor. Esto desencadena el inicio de la acción.
El servidor envía feedback al cliente mientras se está llevando a cabo la acción.
Una vez que la acción finaliza, el servidor devuelve una respuesta al cliente.
```shell
ros2 action list
```
Para llamar a un servidor de acción en ROS 2, se puede utilizar el comando ros2 action send_goal. La estructura del comando es la siguiente:
```shell 
ros2 action send_goal <nombre_de_la_acción> <tipo_de_la_acción> <valores>
```
En ROS 2, las acciones proporcionan feedback y se puede visualizar este feedback. Sin embargo, es necesario especificar que se desea visualizar el feedback al llamar al servidor de acción. El servidor de acción puede enviar feedback al cliente mientras se está llevando a cabo la acción. Una vez que la acción finaliza, el servidor devuelve una respuesta al cliente.
 Puedes agregar la opción -f o --feedback al comando ros2 action send_goal para visualizar el feedback. La opción -f es la forma abreviada de escribir --feedback
 Al llamar a un servidor de acción en ROS 2 implica enviar un objetivo a él. Al igual que los temas y los servicios, el funcionamiento de los servidores de acción se basa en el envío de mensajes. 
 En ROS 2, el mensaje de un servidor de acción se divide en tres partes: el objetivo, el resultado y el feedback. El objetivo describe lo que se espera que haga la acción, el resultado describe el resultado de la acción y el feedback describe el progreso hacia la finalización de la acción
 En ROS 2, el feedback de una acción es un mensaje que el servidor de acción genera ocasionalmente para indicar cómo va la acción y para informar al cliente sobre el estado de la acción solicitada. El feedback se genera mientras la acción está en progreso.
    ROS2 Debugging Messages
    En ROS 2, el sistema de registro admite cinco niveles de gravedad para los mensajes de registro: DEBUG, INFO, WARN, ERROR y FATAL.
    ![image](https://github.com/JERONIMODIFRANCO/SCORBOT-ER-IX-ROS2/assets/95137387/a6e3d2b8-05e4-4408-b5bc-2824b97bdd8f)
    
# HERRAMIENTAS

# RVIZ2
RVIZ2 es una herramienta de visualización que permite ver imágenes, nubes de puntos, láseres, transformaciones cinemáticas, modelos de robots y más. No es una herramienta de simulación, sino que representa lo que se publica en los temas por la simulación o el robot real. 
RVIZ2 es una herramienta de código abierto disponible de forma gratuita y es compatible con ROS 2.
Para abrir la aplicación:
```shell
rviz2
```
![image](https://github.com/JERONIMODIFRANCO/SCORBOT-ER-IX-ROS2/assets/95137387/fe6e1b48-98ba-4c8e-b6cb-3f745f452476)

# tf2_tools view_frames
El programa view_frames es una herramienta muy útil que te permite visualizar un diagrama de árbol que muestra la relación entre los diferentes marcos de referencia de tu robot. Al ejecutar el programa, se generará un archivo PDF en el directorio donde se ejecute. Esta herramienta es útil para comprender la estructura de los marcos de referencia utilizados en tu sistema robótico, lo que puede ser fundamental para el desarrollo y la depuración.
