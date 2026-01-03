#!/bin/bash
# Script para cambiar fácilmente entre FastRTPS y CycloneDDS

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Función de ayuda
show_help() {
    echo "Uso: source setup_rmw.sh [OPCIÓN]"
    echo ""
    echo "Opciones:"
    echo "  cyclonedds    Configurar CycloneDDS como middleware (default)"
    echo "  show          Mostrar configuración actual"
    echo "  help          Mostrar esta ayuda"
    echo ""
    echo "Ejemplo:"
    echo "  source setup_rmw.sh cyclonedds"
    echo ""
}

# Función para mostrar configuración actual
show_config() {
    echo "=========================================="
    echo "  Configuración RMW Actual"
    echo "=========================================="
    echo "  RMW: ${RMW_IMPLEMENTATION:-'default (FastRTPS)'}"
    echo "  Dominio: ${ROS_DOMAIN_ID:-'0 (default)'}"
    echo "  Localhost only: ${ROS_LOCALHOST_ONLY:-'not set'}"
    if [ -n "$CYCLONEDDS_URI" ]; then
        echo "  CycloneDDS config: $CYCLONEDDS_URI"
    fi
    echo "=========================================="
}

# Si no hay argumentos, mostrar ayuda
if [ $# -eq 0 ]; then
    show_help
    return 0 2>/dev/null || exit 0
fi

# Procesar argumento
case "$1" in
    "cyclonedds"|"")
        echo -e "${GREEN}🔄 Configurando CycloneDDS...${NC}"
        
        # Verificar si existe el script de setup
        SCRIPT_DIR="$HOME/ros2_ws/ros2_ws_wsl2_RasPI"
        if [ -f "$SCRIPT_DIR/setup_cyclonedds.sh" ]; then
            source "$SCRIPT_DIR/setup_cyclonedds.sh"
            echo -e "${GREEN}✓ CycloneDDS configurado${NC}"
        else
            echo -e "${RED}✗ Error: No se encontró setup_cyclonedds.sh${NC}"
            echo "  Ubicación esperada: $SCRIPT_DIR/setup_cyclonedds.sh"
            return 1 2>/dev/null || exit 1
        fi
        ;;
        
    "show")
        show_config
        ;;
        
    "help"|"-h"|"--help")
        show_help
        ;;
        
    *)
        echo -e "${RED}✗ Error: Opción no reconocida: $1${NC}"
        echo ""
        show_help
        return 1 2>/dev/null || exit 1
        ;;
esac
