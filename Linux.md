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
```sh
sudo apt install <package> # para forzar instalar el package, sudo tiene los permisos para realizar cambios en el sistema
apt search <package> # para buscar paquetes por su nombre
sudo apt-get remove <package> # para eliminar paquetes
```
```sh
grep # para filtrar en busquedas de otros comandos o buscar dentro de archivos
```
