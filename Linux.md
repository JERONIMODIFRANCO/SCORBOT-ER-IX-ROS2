## LINUX
### bash commands
```sh
cd /home/user/... # para cambiar de directorio, equivalencia a tener en cuenta /home/user/ = ~
cd .. # para ir hacia atras en la jerarquia de las carpetas 
```
```sh
pwd # para mostrar el directorio actual
```
```sh
ls # para mostrar los arhcivos y carpetas dentro del directorio actual
ls -a # ver ademas archivos ocultos
ls --help # info extra acerca del comando, la notacion suele ser asì para todos los comandos
```
```sh
mkdir <folder> # para crear un nuevo directorio/carpeta
```
```sh
touch <filename>.txt # para crear nuevos archivos
```
```sh
vi <filename>.txt # editor visual 
```
```sh
mv <file/folder we want to move> <destination> # para mover archivos
```
```sh
cp <file/folder we want to copy> <name of the new file/folder> # para copiar archivos, agregar -r para carpetas
```
```sh
rm <file to remove> # para eliminar archivos, agregar -r para carpetas
```
```sh
cd # para moverse al directorio raiz
vi .bashrc # para modificar el bashrc, que es un script especial que se ejecuta cada vez que se abre una nueva terminal
source .bashrc # para forzar la ejecucion del script actualizando los cambios
```
```sh
export <env_variable> # para forzar la actualizacion de la variable
```
### sudo y apt

Linux uses a **dpkg** packaging system. A packaging system is a way to provide programs and applications for installation

**APT** (Advanced Package Tool) is the command line tool to interact with this packaging system

apt-get basically works on a database of available packages. If you don’t update this database, the system won’t know if there are newer packages available or not

**Sudo** is a utility of Linux systems (and other Unix based systems) that allows users to execute a program or a command with the privileges (permissions) of another user (usually the **root** user)

```sh
sudo apt install <package> # para forzar instalar el package, sudo tiene los permisos para realizar cambios en el sistema
apt search <package> # para buscar paquetes por su nombre
sudo apt-get remove <package> # para eliminar paquetes
```
```sh
grep # para filtrar en busquedas de otros comandos o buscar dentro de archivos
```
There are 2 types of processes in Linux:

- **Foreground processes** : These are initialized and controlled through a terminal session
- **Background processes**: These are processes not connected to a terminal
```sh
htop # ambos comandos para visualizar procesos
ps # mas sencillo para principiantes
```
```sh
Ctrl+c # para terminiar procesos de terminal o foreground
Ctrl+z # para suspender o enviar al background un proceso, luego debera ser matado con el comando kill
bg # para traer un proceso del reposo
```
```sh
# The PID will be required to terminate the process
kill 26244
```
### SSH protocol
```sh
ssh <user>@<host> # para realizar comunicaciones seguras
```
