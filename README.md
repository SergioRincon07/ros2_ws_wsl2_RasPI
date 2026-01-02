# WSL to Raspberry Pi ROS2 Communication

Proyecto de comunicación ROS2 entre WSL2 (Windows Subsystem for Linux) y Raspberry Pi utilizando mensajes estándar de ROS2.

## Descripción del Proyecto

Este proyecto implementa comunicación básica entre un PC con WSL2 y una Raspberry Pi usando ROS2. Incluye:
- **Talker (Publisher)**: Nodo que publica mensajes desde WSL
- **Listener (Subscriber)**: Nodo que escucha mensajes en la Raspberry Pi
- Uso de mensajes estándar (`std_msgs/String`)

## Estructura del Proyecto

```
ros2_ws_wsl2_RasPI/
├── src/
│   └── wsl_raspi_comm/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── wsl_raspi_comm
│       ├── wsl_raspi_comm/
│       │   ├── __init__.py          # ¡IMPORTANTE! No olvidar este archivo
│       │   ├── talker.py
│       │   └── listener.py
│       └── test/
├── build/
├── install/
└── log/
```

## Requisitos Previos

### En WSL2 y Raspberry Pi:
- Ubuntu 22.04 (recomendado) o compatible
- ROS2 Jazzy (o Humble/Iron)
- Python 3.10+
- Colcon build tool

### Verificar instalación de ROS2:
```bash
ros2 --version
```

## Instalación

### 1. Configurar el workspace

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
```

### 2. Verificar estructura del paquete Python

**IMPORTANTE**: Asegurarse de que existe el archivo `__init__.py`:

```bash
# Verificar que existe
ls src/wsl_raspi_comm/wsl_raspi_comm/__init__.py

# Si no existe, crearlo
touch src/wsl_raspi_comm/wsl_raspi_comm/__init__.py
```

### 3. Limpiar builds anteriores (recomendado)

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
rm -rf build install log
```

### 4. Construir el paquete

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
colcon build
```

### 5. Source del workspace

```bash
source ~/ros2_ws/ros2_ws_wsl2_RasPI/install/setup.bash
```

**Recomendación**: Agregar al `~/.bashrc`:

```bash
echo "source ~/ros2_ws/ros2_ws_wsl2_RasPI/install/setup.bash" >> ~/.bashrc
```

## Configuración para WSL2

### Problema conocido con CycloneDDS en WSL2

En WSL2, CycloneDDS puede tener problemas con las interfaces de red. La solución es usar FastRTPS:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**Configuración permanente** (agregar al `~/.bashrc`):

```bash
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
```

### Verificar variables de entorno ROS2

```bash
printenv | grep -i ROS
```

## Uso del Proyecto

### En WSL2 (Talker/Publisher):

```bash
# Source el workspace
source ~/ros2_ws/ros2_ws_wsl2_RasPI/install/setup.bash

# Configurar FastRTPS (si no está en bashrc)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Ejecutar el talker
ros2 run wsl_raspi_comm talker
```

Salida esperada:
```
[INFO] [wsl_talker]: Hola desde WSL #0
[INFO] [wsl_talker]: Hola desde WSL #1
[INFO] [wsl_talker]: Hola desde WSL #2
...
```

### En Raspberry Pi (Listener/Subscriber):

```bash
# Source el workspace
source ~/ros2_ws/ros2_ws_wsl2_RasPI/install/setup.bash

# Ejecutar el listener
ros2 run wsl_raspi_comm listener
```

## Configuración de Red

### 1. Verificar conectividad

Desde WSL2, hacer ping a la Raspberry Pi:
```bash
ping <IP_RASPBERRY_PI>
```

Desde la Raspberry Pi, hacer ping a WSL2:
```bash
ping <IP_WSL2>
```

Para obtener la IP de WSL2:
```bash
ip addr show eth0
```

### 2. Configurar ROS_DOMAIN_ID (opcional)

Si hay múltiples sistemas ROS2 en la red:

```bash
# En ambos dispositivos, usar el mismo ID
export ROS_DOMAIN_ID=42
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
```

### 3. Verificar interfaces de red

```bash
ip addr show
```

## Verificación y Debugging

### 1. Listar nodos activos:

```bash
ros2 node list
```

Deberías ver:
```
/wsl_talker
/raspi_listener
```

### 2. Listar topics activos:

```bash
ros2 topic list
```

### 3. Ver información de un topic:

```bash
ros2 topic info /chatter -v
```

### 4. Monitorear mensajes en tiempo real:

```bash
ros2 topic echo /chatter
```

### 5. Ver frecuencia de publicación:

```bash
ros2 topic hz /chatter
```

## Solución de Problemas

### Error: `ModuleNotFoundError: No module named 'wsl_raspi_comm'`

**Causa**: Falta el archivo `__init__.py` en el directorio del módulo Python.

**Solución**:
```bash
# Crear el archivo __init__.py
touch src/wsl_raspi_comm/wsl_raspi_comm/__init__.py

# Limpiar y reconstruir
rm -rf build install log
colcon build
source install/setup.bash
```

### Error: CycloneDDS no puede crear nodo en WSL2

**Síntoma**: 
```
[ERROR] [rmw_cyclonedds_cpp]: rmw_create_node: failed to create domain
eth2: does not match an available interface
```

**Solución**: Usar FastRTPS en lugar de CycloneDDS:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 run wsl_raspi_comm talker
```

### El listener no recibe mensajes

1. **Verificar red**: 
   ```bash
   ping <IP_OTRO_DISPOSITIVO>
   ```

2. **Verificar ROS_DOMAIN_ID**: Debe ser igual en ambos dispositivos
   ```bash
   echo $ROS_DOMAIN_ID
   ```

3. **Verificar RMW_IMPLEMENTATION**: Debe ser igual en ambos dispositivos
   ```bash
   echo $RMW_IMPLEMENTATION
   ```

4. **Verificar topics**:
   ```bash
   ros2 topic list
   ros2 node list
   ```

5. **Verificar firewall** (en ambos dispositivos):
   ```bash
   # Desactivar temporalmente para testing
   sudo ufw disable
   
   # O permitir tráfico específico
   sudo ufw allow from <IP_OTRO_DISPOSITIVO>
   ```

### Error al construir el paquete

```bash
# Limpiar completamente y reconstruir
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
rm -rf build install log
colcon build
```

### Verificar instalación del paquete

```bash
# Ver archivos instalados
ls install/wsl_raspi_comm/lib/wsl_raspi_comm/

# Verificar módulo Python
ls install/wsl_raspi_comm/lib/python3.12/site-packages/wsl_raspi_comm/
```

## Comandos Útiles

### Reconstruir después de cambios en código:

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
colcon build --packages-select wsl_raspi_comm
source install/setup.bash
```

### Ver logs de construcción:

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI/log/latest_build
ls -lt
```

### Limpiar workspace completo:

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
rm -rf build install log
```

### Verificar dependencias del paquete:

```bash
rosdep check wsl_raspi_comm
```

## Arquitectura del Sistema

### Talker (Publisher)
- **Nodo**: `wsl_talker`
- **Topic**: `/chatter`
- **Tipo de mensaje**: `std_msgs/String`
- **Frecuencia**: ~1 Hz

### Listener (Subscriber)
- **Nodo**: `raspi_listener`
- **Topic**: `/chatter`
- **Tipo de mensaje**: `std_msgs/String`

## Notas Importantes

1. **__init__.py es obligatorio**: Los paquetes Python en ROS2 requieren un archivo `__init__.py` en el directorio del módulo.

2. **FastRTPS en WSL2**: Se recomienda usar FastRTPS en lugar de CycloneDDS en WSL2 debido a problemas de compatibilidad con las interfaces de red virtuales.

3. **Sincronización de tiempo**: Para comunicación multi-máquina, asegurar que los relojes estén sincronizados (usar NTP).

4. **Network discovery**: ROS2 usa multicast UDP para descubrimiento de nodos. Asegurar que la red lo permita.

## Contacto

Maintainer: Sergio  
Email: sergiorincon50@gmail.com

## Licencia

TODO: License declaration