# Sistema Pan-Tilt con Joystick y Servos MG90S

Este proyecto controla dos servos MG90S de 180 grados usando un joystick analógico para crear un sistema Pan-Tilt. El sistema incluye calibración automática, suavizado de movimientos y **mantiene la última posición** sin volver al centro.

## 🎯 Características Principales

- ✅ **Mantiene la última posición**: Los servos no vuelven al centro cuando sueltas el joystick
- ✅ **Calibración automática**: Detecta automáticamente los rangos del joystick
- ✅ **Zona muerta configurable**: Evita movimientos accidentales
- ✅ **Movimientos suaves**: Sin vibraciones ni movimientos bruscos
- ✅ **Monitor serial detallado**: Información en tiempo real

## 🔧 Componentes Requeridos

- ESP32 (cualquier modelo)
- 2x Servos MG90S (180 grados)
- 1x Joystick analógico (GND, 3V, VRX, VRY, SW)
- Soporte Pan-Tilt para los servos
- Fuente de alimentación de 5V para los servos
- Cables de conexión

## 🔌 Conexiones

### Joystick
- **GND** → GND del ESP32
- **3V** → 3.3V del ESP32
- **VRX** → GPIO 32 (ADC1_CH4)
- **VRY** → GPIO 33 (ADC1_CH5)
- **SW** → GPIO 25

### Servos
- **Servo Pan (Horizontal)**
  - Señal → GPIO 13
  - VCC → 5V (fuente externa recomendada)
  - GND → GND común

- **Servo Tilt (Vertical)**
  - Señal → GPIO 14
  - VCC → 5V (fuente externa recomendada)
  - GND → GND común

## 🚀 Cómo Funciona

### 1. **Al hacer Upload:**
```
=== SISTEMA PAN-TILT CON JOYSTICK ===
Inicializando...
Servos configurados en posición central (90°)
Presiona el botón del joystick para calibrar...
Mueve el joystick en todas las direcciones durante la calibración
================================================
```

### 2. **Calibración (5 segundos):**
```
=== CALIBRACIÓN DEL JOYSTICK ===
Mueve el joystick en todas las direcciones durante 5 segundos
Asegúrate de llegar a los extremos en todas las direcciones
Progreso: 20% | X: 100-3900, Y: 150-3850
Progreso: 40% | X: 50-3950, Y: 100-3900
...
=== CALIBRACIÓN COMPLETADA ===
Eje X: 100 - 3900 (Centro: 2000)
Eje Y: 150 - 3850 (Centro: 2000)
Rango X: 3800, Rango Y: 3700
El sistema está listo para usar!
================================================
```

### 3. **Uso Normal:**
```
Pan: 45° | Tilt: 135° | Joystick X: 1500, Y: 3500 | Zona muerta: No
Pan: 90° | Tilt: 90° | Joystick X: 2000, Y: 2000 | Zona muerta: Sí
```

## ⚙️ Configuración Ajustable

### Pines (modificables en el código)
```cpp
#define JOYSTICK_VRX_PIN 32  // Eje X del joystick
#define JOYSTICK_VRY_PIN 33  // Eje Y del joystick
#define JOYSTICK_SW_PIN  25  // Botón del joystick
#define SERVO_PAN_PIN    13  // Servo horizontal
#define SERVO_TILT_PIN   14  // Servo vertical
```

### Parámetros de Comportamiento
```cpp
#define JOYSTICK_DEADZONE 100     // Zona muerta (0-500)
#define JOYSTICK_SMOOTHING 0.3    // Suavizado (0.0-1.0)
#define SERVO_MIN_ANGLE  0        // Ángulo mínimo
#define SERVO_MAX_ANGLE  180      // Ángulo máximo
#define SERVO_CENTER_ANGLE 90     // Posición central
```

## 🎮 Instrucciones de Uso

1. **Compila y sube el código** al ESP32
2. **Abre el Monitor Serial** (115200 baudios)
3. **Presiona el botón del joystick** para calibrar
4. **Mueve el joystick** en todas las direcciones durante 5 segundos
5. **¡Listo!** Ahora controla los servos:
   - **Eje X**: Control horizontal (Pan) - Izquierda/Derecha
   - **Eje Y**: Control vertical (Tilt) - Arriba/Abajo
   - **Suelta el joystick**: Los servos mantienen su posición

## 🔧 Ajustes Recomendados

### Si los movimientos son muy lentos:
```cpp
#define JOYSTICK_SMOOTHING 0.5  // Aumentar de 0.3 a 0.5
```

### Si los movimientos son muy bruscos:
```cpp
#define JOYSTICK_SMOOTHING 0.1  // Disminuir de 0.3 a 0.1
```

### Si el joystick tiene mucho drift:
```cpp
#define JOYSTICK_DEADZONE 150   // Aumentar de 100 a 150
```

### Si el joystick es muy sensible:
```cpp
#define JOYSTICK_DEADZONE 50    // Disminuir de 100 a 50
```

## ⚠️ Notas Importantes

- **Alimentación de Servos**: Los servos MG90S pueden consumir hasta 500mA cada uno. Se recomienda usar una fuente de alimentación externa de 5V.
- **Calibración**: Si los movimientos no son precisos, recalibra el joystick presionando el botón.
- **Zona Muerta**: El joystick debe estar en la zona muerta para mantener la posición. Si se mueve ligeramente, los servos se moverán.
- **Monitor Serial**: Mantén el monitor serial abierto para ver el estado del sistema.

## 🛠️ Solución de Problemas

### Los servos no se mueven
- Verifica las conexiones de alimentación
- Asegúrate de que los pines de señal estén correctos
- Revisa que la calibración se haya completado

### Movimientos erráticos
- Recalibra el joystick
- Ajusta la zona muerta
- Verifica que no haya interferencias en los cables

### Movimientos muy lentos o rápidos
- Ajusta el factor de suavizado (JOYSTICK_SMOOTHING)
- Modifica el intervalo de actualización (UPDATE_INTERVAL)

### Los servos vuelven al centro
- Verifica que la zona muerta no sea muy grande
- Asegúrate de que el joystick esté bien calibrado
- Revisa que las variables `lastValidPanAngle` y `lastValidTiltAngle` se estén actualizando

## 📊 Información Técnica

- **Frecuencia de actualización**: 50Hz (20ms)
- **Resolución ADC**: 12 bits (0-4095)
- **Rango de servos**: 0° - 180°
- **Posición central**: 90°
- **Tiempo de calibración**: 5 segundos
- **Muestras de calibración**: ~500 muestras 