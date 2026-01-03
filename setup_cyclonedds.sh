#!/bin/bash
# Configuración de variables de entorno para comunicación multi-dispositivo con CycloneDDS
# Adaptado para WSL2 y Raspberry Pi

# Usar CycloneDDS (recomendado para multi-máquina)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Apuntar al archivo de configuración DDS
# Primero intentar el archivo local del proyecto, luego el del home
if [ -f "$HOME/ros2_ws/ros2_ws_wsl2_RasPI/config/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/ros2_ws/ros2_ws_wsl2_RasPI/config/cyclonedds.xml
elif [ -f "$HOME/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
else
    echo "⚠️  Advertencia: No se encontró archivo cyclonedds.xml"
    echo "   El sistema usará configuración por defecto de CycloneDDS"
fi

# Dominio ROS2 (debe ser el mismo en todos los dispositivos)
export ROS_DOMAIN_ID=10

# Configuración de descubrimiento automático (reemplaza ROS_LOCALHOST_ONLY)
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET

# Opcional: para debugging de descubrimiento DDS
# Descomentar la siguiente línea para habilitar trazas detalladas
# export CYCLONEDDS_TRACE=info

echo "=========================================="
echo "  Configuración CycloneDDS cargada"
echo "=========================================="
echo "  RMW: $RMW_IMPLEMENTATION"
echo "  Dominio: $ROS_DOMAIN_ID"
echo "  Config: ${CYCLONEDDS_URI:-'default'}"
echo "  Discovery range: $ROS_AUTOMATIC_DISCOVERY_RANGE"
echo "=========================================="
echo ""
echo "✓ Listo para comunicación multi-dispositivo"
echo ""
