# Comparativa de Proyectos ROS2: Integración de CycloneDDS

**Fecha:** 3 de enero de 2026  
**Proyectos comparados:**
- `~/ros2_ws/ros2_ws_wsl2_RasPI` (Proyecto actual)
- `~/ros2_ws/ros2_ws_cyclonedds` (Proyecto de referencia)

---

## 1. Resumen Ejecutivo

El proyecto `ros2_ws_cyclonedds` cuenta con una infraestructura completa para comunicación multi-dispositivo usando CycloneDDS, mientras que el proyecto actual `ros2_ws_wsl2_RasPI` tiene configuración básica y está actualmente usando FastRTPS debido a problemas con CycloneDDS en WSL2.

### Funcionalidades presentes en ros2_ws_cyclonedds:
✅ Scripts de configuración automatizada para CycloneDDS  
✅ Archivo de configuración XML para CycloneDDS  
✅ Script de diagnóstico de red y comunicación  
✅ Script de construcción automatizada  
✅ Archivos launch para diferentes escenarios de prueba  
✅ Nodos de prueba más completos (publisher, subscriber, ping-pong, servicios)  

### Funcionalidades en ros2_ws_wsl2_RasPI:
⚠️ Nodos básicos (talker/listener)  
⚠️ Documentación sobre problemas con CycloneDDS  
⚠️ Sin scripts de automatización  
⚠️ Sin archivos launch  
⚠️ Sin archivo de configuración CycloneDDS  

---

## 2. Comparativa de Scripts Shell

### 2.1 Scripts en ros2_ws_cyclonedds

#### `setup_multidevice.sh` ⭐
**Ubicación:** `~/ros2_ws/ros2_ws_cyclonedds/setup_multidevice.sh`

**Funcionalidades:**
- Configura `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Define variable `CYCLONEDDS_URI` apuntando a archivo XML de configuración
- Establece `ROS_DOMAIN_ID=10` para comunicación multi-dispositivo
- Configura `ROS_LOCALHOST_ONLY=0` para permitir comunicación de red
- Incluye opción de debugging con `CYCLONEDDS_TRACE`
- Muestra resumen de configuración al cargar

**Contenido:**
```bash
# Usar CycloneDDS (más estable para multi-máquina)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Apuntar al archivo de configuración DDS
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

# Dominio ROS2 (debe ser el mismo en ambas máquinas)
export ROS_DOMAIN_ID=10

# Localhost only OFF para permitir comunicación en red
export ROS_LOCALHOST_ONLY=0

# Opcional: para debugging de descubrimiento
# export CYCLONEDDS_TRACE=info

echo "Configuración DDS cargada:"
echo "  RMW: $RMW_IMPLEMENTATION"
echo "  Dominio: $ROS_DOMAIN_ID"
echo "  Config: $CYCLONEDDS_URI"
```

---

#### `check_connection.sh` ⭐
**Ubicación:** `~/ros2_ws/ros2_ws_cyclonedds/check_connection.sh`

**Funcionalidades:**
- Diagnóstico completo de configuración ROS2
- Verificación de variables de entorno (ROS_DISTRO, ROS_DOMAIN_ID, RMW_IMPLEMENTATION)
- Lista de interfaces de red activas
- Listado de nodos, topics y servicios ROS2 activos
- Información de participantes DDS
- Reporte de configuración de red usando `ros2 doctor`

**Contenido:**
```bash
#!/bin/bash
# Script de diagnóstico para comunicación multi-dispositivo

echo "=== Diagnóstico de Comunicación ROS2 ==="
echo ""

echo "1. Configuración ROS2:"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "   ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo ""

echo "2. Interfaces de red activas:"
ip addr show | grep -E "^[0-9]|inet " | grep -v "127.0.0.1"
echo ""

echo "3. Nodos ROS2 activos:"
ros2 node list
echo ""

echo "4. Topics disponibles:"
ros2 topic list
echo ""

echo "5. Servicios disponibles:"
ros2 service list
echo ""

echo "6. Participantes DDS:"
ros2 daemon stop > /dev/null 2>&1
ros2 doctor --report | grep -A 20 "NETWORK CONFIGURATION"
echo ""

echo "=== Fin del diagnóstico ==="
```

---

#### `build_test_packages.sh` ⭐
**Ubicación:** `~/ros2_ws/ros2_ws_cyclonedds/build_test_packages.sh`

**Funcionalidades:**
- Compilación automatizada de paquetes de prueba
- Creación de symlinks para compatibilidad con launch files
- Verificación de compilación exitosa
- Instrucciones post-instalación

**Contenido:**
```bash
#!/bin/bash
# Script para construir los paquetes de prueba

echo "🔨 Compilando paquetes de prueba..."

cd /home/sergio/ros2_ws

# Compilar
colcon build --packages-select test_comm_msgs test_comm_nodes --symlink-install

# Verificar resultado
if [ $? -eq 0 ]; then
    echo "✓ Compilación exitosa"
    
    # Crear symlinks para compatibilidad con launch files
    echo "🔗 Creando symlinks de ejecutables..."
    mkdir -p install/test_comm_nodes/lib/test_comm_nodes
    cd install/test_comm_nodes/lib/test_comm_nodes
    ln -sf ../../bin/* . 2>/dev/null
    cd /home/sergio/ros2_ws
    
    echo "✓ Setup completo"
    echo ""
    echo "Para usar los paquetes, ejecuta:"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source /home/sergio/ros2_ws/install/setup.bash"
    echo "  source /home/sergio/ros2_ws/setup_multidevice.sh"
else
    echo "❌ Error en la compilación"
    exit 1
fi
```

---

### 2.2 Scripts en ros2_ws_wsl2_RasPI

**Estado actual:** ❌ No existen scripts shell personalizados de configuración o automatización.

**Scripts presentes:** Solo scripts generados automáticamente por colcon en directorios `build/` e `install/`:
- `colcon_command_prefix_setup_py.sh`
- `setup.sh`, `local_setup.sh`
- Hooks estándar de ROS2

---

## 3. Archivo de Configuración CycloneDDS XML

### 3.1 Configuración en ros2_ws_cyclonedds

**Ubicación:** `~/cyclonedds.xml` (referenciado desde `setup_multidevice.sh`)

**Características:**
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
    <Domain id="any">
        <General>
            <Interfaces>
                <NetworkInterface name="eth2" priority="10" />
                <NetworkInterface name="eth0" priority="5" />
                <NetworkInterface name="wlan0" priority="5" />
            </Interfaces>
            <!-- Configuración para comunicación entre dispositivos -->
            <AllowMulticast>true</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
            <FragmentSize>4000B</FragmentSize>
        </General>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
        </Discovery>
        <Tracing>
            <!-- Habilitar para debugging (info), reducir a warning en producción -->
            <Verbosity>warning</Verbosity>
            <OutputFile>stdout</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
```

**Funcionalidades configuradas:**
- ✅ Priorización de interfaces de red (eth2 > eth0 = wlan0)
- ✅ Multicast habilitado para descubrimiento automático
- ✅ Tamaño máximo de mensaje: 65.5 KB
- ✅ Fragmentación de mensajes grandes: 4 KB
- ✅ Descubrimiento automático de participantes
- ✅ Soporte para hasta 100 participantes
- ✅ Logging configurable (warning por defecto)

---

### 3.2 Configuración en ros2_ws_wsl2_RasPI

**Estado actual:** ❌ No existe archivo de configuración CycloneDDS XML

**Documentación existente:** En README.md se menciona problema con CycloneDDS en WSL2 y se recomienda usar FastRTPS:
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## 4. Comparativa de Estructura de Paquetes

### 4.1 Paquetes en ros2_ws_cyclonedds

**Paquetes:**
1. `test_comm_msgs` - Mensajes personalizados
2. `test_comm_nodes` - Nodos de prueba

**Nodos disponibles en test_comm_nodes:**
- `publisher_node` - Publicador genérico
- `subscriber_node` - Suscriptor genérico
- `service_server` - Servidor de servicios
- `service_client` - Cliente de servicios
- `ping_node` - Nodo ping para latencia
- `pong_node` - Nodo pong para latencia

**Launch files disponibles:**
- `publisher_only.launch.py` - Solo publicador
- `subscriber_only.launch.py` - Solo suscriptor
- `test_pubsub.launch.py` - Publisher y subscriber juntos
- `test_service.launch.py` - Cliente-servidor
- `test_pingpong.launch.py` - Prueba de latencia

**Características del setup.py:**
```python
data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
],
```
- ✅ Soporte para archivos launch
- ✅ Soporte para archivos de configuración YAML
- ✅ 6 entry points configurados

---

### 4.2 Paquetes en ros2_ws_wsl2_RasPI

**Paquetes:**
1. `wsl_raspi_comm` - Comunicación básica

**Nodos disponibles:**
- `talker` - Publicador simple
- `listener` - Suscriptor simple

**Launch files:** ❌ No existen

**Características del setup.py:**
- ❌ Sin soporte para launch files
- ❌ Sin soporte para archivos de configuración
- ⚠️ Solo 2 entry points básicos

---

## 5. Documentación

### 5.1 README en ros2_ws_wsl2_RasPI

**Fortalezas:**
- ✅ Documentación detallada de problemas con CycloneDDS en WSL2
- ✅ Instrucciones de instalación paso a paso
- ✅ Solución de problemas comunes
- ✅ Comandos de verificación y debugging

**Debilidades:**
- ⚠️ Enfocado en FastRTPS como solución
- ⚠️ No incluye configuración avanzada de CycloneDDS
- ⚠️ No documenta comunicación multi-dispositivo con CycloneDDS

---

### 5.2 README en ros2_ws_cyclonedds

**Estado:** No se ha verificado la existencia o contenido de README.

---

## 6. Recomendaciones de Integración

### 6.1 Prioridad Alta ⭐⭐⭐

#### 1. Integrar `setup_multidevice.sh`
**Acción:** Crear script de configuración para CycloneDDS en el proyecto actual.

**Beneficios:**
- Configuración automatizada de variables de entorno
- Facilita cambio entre FastRTPS y CycloneDDS
- Estandariza configuración entre dispositivos

**Implementación sugerida:**
```bash
# Crear archivo en el workspace
~/ros2_ws/ros2_ws_wsl2_RasPI/setup_cyclonedds.sh
```

---

#### 2. Crear archivo `cyclonedds.xml`
**Acción:** Copiar y adaptar la configuración XML de CycloneDDS.

**Adaptaciones necesarias para WSL2:**
```xml
<Interfaces>
    <NetworkInterface name="eth0" priority="10" />
    <NetworkInterface name="wlan0" priority="5" />
    <!-- Evitar interfaces virtuales problemáticas de WSL2 -->
</Interfaces>
```

**Ubicación sugerida:**
- `~/cyclonedds.xml` (para uso global)
- `~/ros2_ws/ros2_ws_wsl2_RasPI/config/cyclonedds.xml` (para el proyecto)

---

#### 3. Integrar `check_connection.sh`
**Acción:** Añadir script de diagnóstico al proyecto actual.

**Beneficios:**
- Facilita debugging de problemas de red
- Verifica configuración de DDS
- Documenta estado del sistema

---

### 6.2 Prioridad Media ⭐⭐

#### 4. Crear script de construcción automatizada
**Acción:** Crear `build_workspace.sh` similar a `build_test_packages.sh`.

**Contenido sugerido:**
```bash
#!/bin/bash
echo "🔨 Compilando workspace..."
cd ~/ros2_ws/ros2_ws_wsl2_RasPI
colcon build --symlink-install
if [ $? -eq 0 ]; then
    echo "✓ Compilación exitosa"
    echo "Para usar ejecuta:"
    echo "  source install/setup.bash"
else
    echo "❌ Error en la compilación"
    exit 1
fi
```

---

#### 5. Añadir archivos launch
**Acción:** Crear directorio `launch/` en el paquete y añadir archivos launch básicos.

**Launch files sugeridos:**
- `talker_only.launch.py`
- `listener_only.launch.py`
- `talker_listener.launch.py`

**Modificar setup.py:**
```python
from glob import glob
import os

data_files=[
    # ... existentes ...
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
],
```

---

### 6.3 Prioridad Baja ⭐

#### 6. Expandir nodos de prueba
**Acción:** Añadir nodos adicionales como ping-pong y servicios.

**Beneficios:**
- Pruebas más completas de comunicación
- Medición de latencia
- Prueba de servicios ROS2

---

#### 7. Añadir directorio config/
**Acción:** Crear directorio para archivos de configuración YAML.

**Uso:**
- Parámetros de nodos
- Configuraciones de red
- Perfiles de dispositivos

---

## 7. Plan de Implementación

### Fase 1: Scripts de Configuración (1-2 horas)
1. ✅ Crear `setup_cyclonedds.sh` en el workspace
2. ✅ Crear archivo `cyclonedds.xml` adaptado para WSL2
3. ✅ Crear `check_connection.sh` para diagnósticos
4. ✅ Actualizar README.md con instrucciones de uso

### Fase 2: Automatización (30 min - 1 hora)
1. ✅ Crear `build_workspace.sh`
2. ✅ Crear script de limpieza `clean_workspace.sh`
3. ✅ Documentar scripts en README

### Fase 3: Launch Files (1-2 horas)
1. ⏳ Crear directorio `src/wsl_raspi_comm/launch/`
2. ⏳ Implementar launch files básicos
3. ⏳ Modificar setup.py
4. ⏳ Reconstruir paquete

### Fase 4: Testing y Validación (1-2 horas)
1. ⏳ Probar scripts en WSL2
2. ⏳ Verificar comunicación con CycloneDDS
3. ⏳ Documentar resultados
4. ⏳ Comparar rendimiento FastRTPS vs CycloneDDS

---

## 8. Consideraciones Especiales para WSL2

### Problema conocido: Interfaces de red en WSL2
CycloneDDS tiene dificultades con las interfaces de red virtuales de WSL2, especialmente `eth2` que es creada dinámicamente.

**Soluciones posibles:**

#### Opción A: Configuración Específica de Interfaces
```xml
<Interfaces>
    <!-- Especificar solo interfaces estables -->
    <NetworkInterface name="eth0" />
</Interfaces>
```

#### Opción B: Forzar IPv4
```xml
<General>
    <Transport>
        <EnableIPv4>true</EnableIPv4>
        <EnableIPv6>false</EnableIPv6>
    </Transport>
</General>
```

#### Opción C: Mantener FastRTPS para WSL2
- Usar CycloneDDS en Raspberry Pi
- Usar FastRTPS en WSL2
- Ambos son compatibles entre sí

---

## 9. Comparativa de Configuración RMW

### CycloneDDS (ros2_ws_cyclonedds)
**Ventajas:**
- ✅ Mejor descubrimiento automático en redes complejas
- ✅ Menor latencia en comunicación multi-dispositivo
- ✅ Mejor manejo de QoS
- ✅ Configuración granular vía XML

**Desventajas:**
- ⚠️ Problemas con WSL2
- ⚠️ Requiere configuración adicional
- ⚠️ Más complejo de debuggear

### FastRTPS (ros2_ws_wsl2_RasPI actual)
**Ventajas:**
- ✅ Funciona bien en WSL2
- ✅ Configuración más simple
- ✅ Bien documentado

**Desventajas:**
- ⚠️ Puede tener problemas de descubrimiento
- ⚠️ Mayor latencia en algunos casos
- ⚠️ Menos opciones de configuración

---

## 10. Conclusiones Finales

### Resumen de Diferencias Clave

| Característica | ros2_ws_cyclonedds | ros2_ws_wsl2_RasPI |
|----------------|-------------------|-------------------|
| Scripts de configuración | ✅ Completo | ❌ Ninguno |
| Archivo CycloneDDS XML | ✅ Presente | ❌ Ausente |
| Script de diagnóstico | ✅ Presente | ❌ Ausente |
| Launch files | ✅ 5 archivos | ❌ Ninguno |
| Nodos de prueba | ✅ 6 nodos | ⚠️ 2 nodos básicos |
| Automatización | ✅ Scripts build | ❌ Manual |
| RMW configurado | CycloneDDS | FastRTPS |
| Documentación | ⚠️ No verificado | ✅ Completo |

---

### Valor Agregado de la Integración

#### Para Desarrollo:
1. **Configuración más rápida** - Scripts automatizados
2. **Debugging más fácil** - Herramientas de diagnóstico
3. **Mayor flexibilidad** - Launch files parametrizables

#### Para Producción:
1. **Mejor rendimiento** - CycloneDDS optimizado
2. **Configuración robusta** - XML bien estructurado
3. **Monitoreo mejorado** - Scripts de verificación

#### Para Mantenimiento:
1. **Estandarización** - Configuración consistente
2. **Reproducibilidad** - Scripts documentados
3. **Troubleshooting** - Diagnósticos automatizados

---

### Recomendación Final

**Enfoque híbrido sugerido:**

1. **Mantener FastRTPS como predeterminado en WSL2** para estabilidad
2. **Integrar scripts y configuración de CycloneDDS** como opción avanzada
3. **Crear switch fácil** entre ambos RMW mediante scripts
4. **Documentar ambas configuraciones** en README
5. **Usar CycloneDDS en Raspberry Pi** para mejor rendimiento

**Script de switch sugerido:**
```bash
# setup_rmw.sh
case "$1" in
    "cyclonedds")
        source setup_cyclonedds.sh
        ;;
    "fastrtps"|*)
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export ROS_DOMAIN_ID=0
        export ROS_LOCALHOST_ONLY=0
        echo "Usando FastRTPS"
        ;;
esac
```

---

### Próximos Pasos Inmediatos

1. ✅ **Crear `setup_cyclonedds.sh`** en el workspace actual
2. ✅ **Crear `cyclonedds.xml`** adaptado para WSL2
3. ✅ **Integrar `check_connection.sh`**
4. ⏳ **Probar CycloneDDS en WSL2** con nueva configuración
5. ⏳ **Documentar resultados** en README
6. ⏳ **Implementar launch files** si pruebas son exitosas

---

**Documento generado el:** 3 de enero de 2026  
**Autor:** GitHub Copilot  
**Versión:** 1.0
