# WSL2 ↔ Raspberry Pi – ROS 2 Jazzy

Comunicación básica ROS 2 entre un PC con WSL2 y una Raspberry Pi usando nodos Python (talker/listener) y mensajes estándar `std_msgs/String`.

---
## 1. Requisitos

En **WSL2** y en **Raspberry Pi**:
- Ubuntu 24.04 (o compatible)
- ROS 2 **Jazzy** instalado y funcional (`ros2 --version`)
- Python ≥ 3.10
- Herramienta de compilación `colcon`

---
## 2. Estructura del workspace

```text
ros2_ws_wsl2_RasPI/
├── config/
│   └──cyclonedds.xml
├── src/
│   └── wsl_raspi_comm/
│       ├── package.xml
│       ├── setup.py / setup.cfg
│       ├── wsl_raspi_comm/
│       │   ├── __init__.py
│       │   ├── talker.py
│       │   └── listener.py
│       └── test/
├── config/cyclonedds.xml
├── build_workspace.sh
├── clean_workspace.sh
├── check_connection.sh
└── setup_cyclonedds.sh
```

---
## 3. Puesta en marcha rápida

### 3.1. Compilar el workspace

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
./build_workspace.sh   # o: colcon build
```

### 3.2. Configurar CycloneDDS (middleware)

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
source setup_cyclonedds.sh
```

### 3.3. Cargar entorno ROS 2 + paquete

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
source install/setup.bash
```

Opcional (para hacerlo automático):
```bash
echo "source ~/ros2_ws/ros2_ws_wsl2_RasPI/install/setup.bash" >> ~/.bashrc
```

Esto configura, entre otros:
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI` → `/home/sergio/ros2_ws/ros2_ws_wsl2_RasPI/config/cyclonedds_WSL.xml` o `/home/sergio/ros2_ws/ros2_ws_wsl2_RasPI/config/cyclonedds_RasPI.xml`
- `ROS_DOMAIN_ID=10`

---
## 4. Ejecutar nodos

### 4.1.1 En WSL2 – Talker (Publisher)

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
source install/setup.bash
source setup_cyclonedds.sh

ros2 run wsl_raspi_comm talker
```

Salida esperada (ejemplo):
```text
[INFO] [wsl_talker]: Hola desde WSL #0
[INFO] [wsl_talker]: Hola desde WSL #1
...
```

### 4.1.2 En Raspberry Pi – Listener (Subscriber)

```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
source install/setup.bash
source setup_cyclonedds.sh

ros2 run wsl_raspi_comm listener
```

Topic usado: `/chatter`  
Tipo de mensaje: `std_msgs/msg/String`

---

### 4.2.1 Prueba Bidireccional - Ping-Pong

**En Linux: Ping**
```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
source install/setup.bash
source setup_cyclonedds.sh

ros2 run wsl_raspi_comm ping_node
```

**En Raspberry Pi: Pong**
```bash
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
source install/setup.bash
source setup_cyclonedds.sh

ros2 run wsl_raspi_comm pong_node
```

## 5. Configuración de red mínima

En **ambos** dispositivos:

1. **Comprobar ping entre WSL2 y Raspberry Pi**
   ```bash
   # Desde WSL2
   ping <IP_RASPBERRY> (En mi caso 192.168.1.6)

   # Desde Raspberry
   ping <IP_WSL2> (En mi caso 192.168.1.7)
   ```

2. **Mismo dominio ROS** (opcional pero recomendado si hay más sistemas ROS 2):
   ```bash
   export ROS_DOMAIN_ID=10
   echo "export ROS_DOMAIN_ID=10" >> ~/.bashrc
   ```

3. **Mismo middleware**:
   ```bash
   echo $RMW_IMPLEMENTATION   # debería mostrar: rmw_cyclonedds_cpp
   ```

4. **CycloneDDS**: configuración en `config/cyclonedds.xml`.
   - Ajusta las interfaces (`eth0`, `wlan0`, etc.) según tu sistema.

---
## 6. Scripts útiles

Desde la raíz del workspace:

- `./build_workspace.sh`  – Compila el workspace (con opción de limpiar antes).
- `./clean_workspace.sh`  – Borra `build/`, `install/` y `log/`.
- `./check_connection.sh` – Diagnóstico rápido de red + ROS 2.
- `source setup_cyclonedds.sh` – Configura CycloneDDS y variables de entorno.

---
## 7. Problemas frecuentes (resumen)

- **`ModuleNotFoundError: No module named 'wsl_raspi_comm'`**
  - Verificar que existe `src/wsl_raspi_comm/wsl_raspi_comm/__init__.py`.
  - Limpiar y recompilar:
    ```bash
    rm -rf build install log
    colcon build
    source install/setup.bash
    ```

- **No hay comunicación entre WSL2 y Raspberry**
  - Comprobar ping en ambos sentidos.
  - Verificar `ROS_DOMAIN_ID` y `RMW_IMPLEMENTATION` son iguales.
  - Comprobar topics y nodos:
    ```bash
    ros2 node list
    ros2 topic list
    ros2 topic echo /chatter
    ```

- **Errores de interfaz en CycloneDDS** (por ejemplo, interfaz que no existe)
  - Editar `config/cyclonedds.xml` y ajustar `<NetworkInterface name="..." />`.
  - Si usas WSL2, normalmente la interfaz principal es `eth0`.

---
## 8. Licencia

TODO: License declaration
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

### En WSL2:
- Ubuntu 24.04 
- ROS2 Jazzy
- Python 3.10+
- Colcon build tool

### En Raspberry Pi:
- Ubuntu 22.04 
- ROS2 Humble
- Python 3.10+
- Colcon build tool
