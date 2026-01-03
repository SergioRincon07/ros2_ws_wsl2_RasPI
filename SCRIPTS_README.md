# Scripts de Configuración - Guía Rápida

## ✅ Fase 1 Completada

Se han creado los siguientes scripts en el workspace:

### 📋 Scripts Creados

1. **setup_cyclonedds.sh** (1.5 KB)
   - Configura todas las variables de entorno para CycloneDDS
   - Establece ROS_DOMAIN_ID=10
   - Apunta al archivo de configuración XML

2. **setup_rmw.sh** (2.1 KB)
   - Script helper para configurar CycloneDDS fácilmente
   - Incluye comando `show` para ver configuración actual
   - Opción de ayuda integrada

3. **check_connection.sh** (3.8 KB)
   - Diagnóstico completo de red y configuración ROS2
   - Verifica interfaces de red, nodos, topics y servicios
   - Incluye diagnóstico DDS y recomendaciones

4. **build_workspace.sh** (2.4 KB)
   - Compilación automatizada del workspace
   - Opción de limpieza antes de compilar
   - Mensajes coloridos e instrucciones post-compilación

5. **clean_workspace.sh** (2.3 KB)
   - Limpieza segura del workspace
   - Muestra tamaño de directorios antes de eliminar
   - Requiere confirmación del usuario

### 📁 Archivos de Configuración

**config/cyclonedds.xml**
- Configuración optimizada para WSL2
- Interfaces de red configuradas (eth0, wlan0)
- IPv4 forzado para mejor compatibilidad
- Multicast habilitado
- Logging configurable

## 🚀 Uso Rápido

### Primer Uso

```bash
# 1. Compilar el workspace
./build_workspace.sh

# 2. Source del workspace
source install/setup.bash

# 3. Configurar CycloneDDS
source setup_cyclonedds.sh

# 4. Verificar todo está OK
./check_connection.sh
```

### Uso Diario

```bash
# En cada terminal nueva:
source install/setup.bash
source setup_cyclonedds.sh

# Ejecutar nodos
ros2 run wsl_raspi_comm talker   # En WSL2
ros2 run wsl_raspi_comm listener # En Raspberry Pi
```

### Automatización en ~/.bashrc

```bash
# Agregar estas líneas a ~/.bashrc para configuración automática:
echo "source ~/ros2_ws/ros2_ws_wsl2_RasPI/install/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/ros2_ws_wsl2_RasPI/setup_cyclonedds.sh" >> ~/.bashrc
```

## 🔧 Comandos Útiles

### Ver configuración actual
```bash
source setup_rmw.sh show
```

### Limpiar y recompilar
```bash
./clean_workspace.sh
./build_workspace.sh
```

### Diagnóstico de problemas
```bash
./check_connection.sh
```

### Ver nodos y topics
```bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
```

## 📝 Próximos Pasos

Ver [COMPARATIVA_CYCLONEDDS.md](COMPARATIVA_CYCLONEDDS.md) para:
- ✅ Fase 1: Scripts de Configuración (COMPLETADA)
- ✅ Fase 2: Automatización (COMPLETADA)
- ⏳ Fase 3: Launch Files (pendiente)
- ⏳ Fase 4: Testing y Validación (pendiente)

## 💡 Notas Importantes

- **Todos los scripts son ejecutables** (chmod +x aplicado)
- **CycloneDDS es el middleware predeterminado**
- **ROS_DOMAIN_ID=10** para comunicación multi-dispositivo
- **Configuración XML en** `config/cyclonedds.xml`

## 🐛 Troubleshooting

### Error de interfaces de red
Si ves errores sobre interfaces de red:
1. Ejecuta `ip addr show` para ver tus interfaces
2. Edita `config/cyclonedds.xml` y ajusta las interfaces
3. Recarga: `source setup_cyclonedds.sh`

### No se ven nodos de otros dispositivos
1. Verifica que ROS_DOMAIN_ID sea igual: `echo $ROS_DOMAIN_ID`
2. Verifica conectividad: `ping <IP_OTRO_DISPOSITIVO>`
3. Ejecuta diagnóstico: `./check_connection.sh`
4. Verifica firewall/red permite multicast UDP

### Problemas de compilación
1. Limpia: `./clean_workspace.sh`
2. Recompila: `./build_workspace.sh`
3. Si persiste, verifica dependencias: `rosdep check wsl_raspi_comm`

---

**Fecha de creación:** 3 de enero de 2026  
**Estado:** Fase 1 y 2 completadas ✅
