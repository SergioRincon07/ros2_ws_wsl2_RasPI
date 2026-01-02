# ROS2 Cross-Distro Communication Demo

This project demonstrates communication between a Linux PC and a Raspberry Pi using ROS2 Humble. It includes a publisher node running on the PC and a subscriber node running on the Raspberry Pi, utilizing a custom message type.

## Project Structure

```
ros2_ws_wsl2_RasPI/
├── src/
│   ├── cross_distro_demo/
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── setup.py
│   │   ├── cross_distro_demo/
│   │   │   ├── __init__.py
│   │   │   ├── pc_publisher.py
│   │   │   ├── pi_subscriber.py
│   │   │   └── message_relay.py
│   │   ├── launch/
│   │   │   ├── pc_launch.py
│   │   │   └── pi_launch.py
│   │   └── config/
│   │       └── params.yaml
│   └── custom_interfaces/
│       ├── package.xml
│       ├── CMakeLists.txt
│       └── msg/
│           └── CrossDistroMessage.msg
├── install/
├── build/
├── log/
├── .gitignore
└── README.md
```

## Descripción del Proyecto

Este proyecto demuestra la comunicación entre dispositivos usando ROS2 Humble con:
- **Mensajes personalizados**: `CrossDistroMessage` con campos de string, float64 y bool
- **Publisher**: Nodo que publica mensajes desde el PC
- **Subscriber**: Nodo que recibe mensajes en la Raspberry Pi
- **QoS configurables**: Perfiles de calidad de servicio ajustables para diferentes condiciones de red

## Requisitos Previos

### En ambos dispositivos (PC y Raspberry Pi):
- Ubuntu 22.04 (Jammy) o compatible
- ROS2 Humble instalado
- Python 3.10+
- Colcon build tool
- Git

### Verificar instalación de ROS2:
```bash
ros2 --version
```

## Instalación y Configuración

### 1. Clonar el repositorio

En ambos dispositivos (PC y Raspberry Pi):

```bash
# Crear workspace si no existe
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clonar el repositorio
git clone <repository-url> .
cd ~/ros2_ws
```

### 2. Instalar dependencias

```bash
cd ~/ros2_ws

# Actualizar rosdep
sudo rosdep init  # Solo si es primera vez
rosdep update

# Instalar dependencias del proyecto
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Construir el workspace

```bash
cd ~/ros2_ws

# Limpiar builds anteriores (opcional)
rm -rf build install log

# Construir todos los paquetes
colcon build

# O construir con output visible para debugging
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Nota importante**: Debes construir primero el paquete `custom_interfaces` antes de `cross_distro_demo`:

```bash
# Construcción por separado (recomendado en primera instalación)
colcon build --packages-select custom_interfaces
colcon build --packages-select cross_distro_demo
```

### 4. Source del workspace

Después de construir, source el workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

**Recomendación**: Agregar al `.bashrc` para que se ejecute automáticamente:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Configuración de Red

Para que los dispositivos se comuniquen, deben estar en la misma red:

### 1. Verificar conectividad

Desde el PC, hacer ping a la Raspberry Pi:
```bash
ping <IP_RASPBERRY_PI>
```

### 2. Configurar ROS_DOMAIN_ID (opcional)

Si hay múltiples sistemas ROS2 en la red:

```bash
# En ambos dispositivos, usar el mismo ID
export ROS_DOMAIN_ID=42
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

### 3. Configurar DDS (opcional pero recomendado)

Para mejor comunicación entre dispositivos:

```bash
# En ambos dispositivos
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Instalar CycloneDDS si no está instalado
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## Uso del Proyecto

### Opción 1: Usando Launch Files (Recomendado)

#### En el PC (Publisher):

```bash
# Source el workspace
source ~/ros2_ws/install/setup.bash

# Lanzar el nodo publisher
ros2 launch cross_distro_demo pc_launch.py
```

#### En la Raspberry Pi (Subscriber):

```bash
# Source el workspace
source ~/ros2_ws/install/setup.bash

# Lanzar el nodo subscriber
ros2 launch cross_distro_demo pi_launch.py
```

### Opción 2: Ejecutar nodos individualmente

#### Publisher (PC):

```bash
ros2 run cross_distro_demo pc_publisher
```

#### Subscriber (Raspberry Pi):

```bash
ros2 run cross_distro_demo pi_subscriber
```

## Verificación y Debugging

### 1. Verificar que los nodos están corriendo:

```bash
ros2 node list
```

Deberías ver:
```
/pc_publisher
/pi_subscriber
```

### 2. Ver topics activos:

```bash
ros2 topic list
```

Deberías ver el topic de comunicación.

### 3. Ver información del mensaje:

```bash
ros2 interface show custom_interfaces/msg/CrossDistroMessage
```

### 4. Monitorear mensajes en tiempo real:

```bash
ros2 topic echo <nombre_del_topic>
```

### 5. Ver información de QoS:

```bash
ros2 topic info <nombre_del_topic> --verbose
```

## Configuración Avanzada

### Modificar parámetros QoS

Editar el archivo [src/cross_distro_demo/config/params.yaml](src/cross_distro_demo/config/params.yaml):

```yaml
pc_publisher:
  ros__parameters:
    qos_profile:
      history: keep_last        # keep_last o keep_all
      depth: 10                 # número de mensajes en buffer
      reliability: reliable     # reliable o best_effort
      durability: transient_local  # transient_local o volatile
```

**Opciones de QoS**:
- **History**: `keep_last` (mantiene últimos N), `keep_all` (mantiene todos)
- **Reliability**: `reliable` (garantiza entrega), `best_effort` (más rápido, puede perder)
- **Durability**: `transient_local` (late joiners reciben último), `volatile` (solo mensajes nuevos)

## Solución de Problemas

### Error: "The install directory 'install' was created with the layout 'merged'"

Si ves este error al ejecutar `colcon build`, significa que el directorio `install` fue creado con un layout incompatible. **Solución**:

```bash
# Limpiar completamente los directorios de compilación
cd ~/ros2_ws
rm -rf build install log

# Reconstruir con symlink-install
colcon build --symlink-install
```

### El subscriber no recibe mensajes:

1. **Verificar red**: Asegurar que ambos dispositivos se pueden hacer ping
2. **Verificar firewall**: 
   ```bash
   sudo ufw allow from <IP_OTRO_DISPOSITIVO>
   ```
3. **Verificar ROS_DOMAIN_ID**: Debe ser igual en ambos dispositivos
4. **Verificar topics**: 
   ```bash
   ros2 topic list
   ros2 topic info <topic_name>
   ```

### Error al construir:

```bash
# Limpiar y reconstruir
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select custom_interfaces
source install/setup.bash
colcon build --packages-select cross_distro_demo

cd /home/sergio/ros2_ws/ros2_ws_wsl2_RasPI && rm -rf build/ install/ log/

```

### Mensaje personalizado no encontrado:

```bash
# Asegurar que custom_interfaces está construido y sourced
cd ~/ros2_ws
colcon build --packages-select custom_interfaces
source install/setup.bash
```

## Comandos Útiles

### Reconstruir después de cambios en código:

```bash
cd ~/ros2_ws
colcon build --packages-select cross_distro_demo
source install/setup.bash
```

### Ver logs:

```bash
cd ~/ros2_ws/log
ls -lt  # ver logs más recientes
```

### Limpiar workspace completo:

```bash
cd ~/ros2_ws
rm -rf build install log
```

## Estructura del Mensaje Personalizado

El mensaje `CrossDistroMessage.msg` contiene:

```
string data      # Datos de texto
float64 value    # Valor numérico
bool status      # Estado booleano
```

## Contribuir

Para contribuir al proyecto:

1. Fork el repositorio
2. Crear una rama para tu feature (`git checkout -b feature/nueva-funcionalidad`)
3. Commit tus cambios (`git commit -m 'Agregar nueva funcionalidad'`)
4. Push a la rama (`git push origin feature/nueva-funcionalidad`)
5. Abrir un Pull Request

## Licencia

Apache License 2.0

## Contacto

Maintainer: Sergio - sergiorincon50@gmail.com

## Notas Adicionales

- **Latencia de red**: Para redes con alta latencia, considerar usar QoS `best_effort` con `volatile`
- **Seguridad**: Para producción, considerar usar SROS2 (Secure ROS2)
- **Performance**: Monitorear con `ros2 topic hz <topic>` para verificar frecuencia de publicación
- **Multi-máquina**: Asegurar que los relojes estén sincronizados (usar NTP)