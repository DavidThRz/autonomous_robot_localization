# IMU Tilt Compensation — Compensación de Inclinación del Acelerómetro

## Resumen

Este documento describe el sistema de compensación de tilt (inclinación de montaje) implementado en el nodo IMU (`imu_node`) del paquete `autonomous_robot_localization_pkg`. El objetivo es proyectar las lecturas de aceleración al plano horizontal real, eliminando la componente de gravedad que se filtra a los ejes X e Y cuando la IMU ADIS16460 no está perfectamente nivelada.

---

## 1. Problema

La IMU está montada físicamente en el robot con una ligera inclinación respecto al plano horizontal. Aunque esta inclinación sea muy pequeña (< 5°), tiene un efecto significativo:

| Inclinación | Aceleración espuria en X/Y | Efecto tras 10s de integración |
|:-----------:|:--------------------------:|:------------------------------:|
| 0.5°        | 0.086 m/s²                 | 4.3 m de error en posición     |
| 1.0°        | 0.171 m/s²                 | 8.6 m de error en posición     |
| 2.0°        | 0.342 m/s²                 | 17.1 m de error en posición    |
| 5.0°        | 0.855 m/s²                 | 42.8 m de error en posición    |

El EKF utiliza `linear_acceleration.x` y `linear_acceleration.y` como entradas de control para propagar velocidad y posición. Cualquier componente de gravedad que aparezca en estos ejes se integra como una aceleración real del robot, produciendo una deriva cuadrática en posición.

---

## 2. Fundamento Matemático

### 2.1. Proyección de la gravedad

Con el sensor estático, la única fuerza medida es la gravedad **g**. Si el sensor tiene una inclinación de **roll (φ)** alrededor del eje X y **pitch (θ)** alrededor del eje Y respecto al plano horizontal, las componentes medidas son:

```
Ax = -g · sin(θ)
Ay =  g · sin(φ) · cos(θ)
Az =  g · cos(φ) · cos(θ)
```

### 2.2. Cálculo de Roll y Pitch

De las ecuaciones anteriores se derivan las fórmulas de inversión:

**Roll (φ):**
```
tan(φ) = Ay / Az
φ = atan2(Ay, Az)
```

**Pitch (θ):**
```
tan(θ) = -Ax / sqrt(Ay² + Az²)
θ = atan2(-Ax, sqrt(Ay² + Az²))
```

> **Nota sobre la fórmula de pitch:** Se usa `sqrt(Ay² + Az²)` en el denominador en lugar de solo `Az`. Esto hace la fórmula estable para todo el rango de roll (evita la singularidad cuando roll ≈ ±90°). Para inclinaciones pequeñas (< 5°), ambas variantes son numéricamente equivalentes, pero usamos la forma más robusta.

> **Nota sobre `atan2`:** Se utiliza `atan2(y, x)` en lugar de `atan(y/x)` porque resuelve correctamente los cuatro cuadrantes y evita divisiones por cero.

### 2.3. Matriz de Rotación

Una vez conocidos roll (φ) y pitch (θ), construimos la matriz de rotación que transforma del frame del sensor al frame horizontal:

```
R = Ry(θ) · Rx(φ)
```

Donde:
```
        ┌ 1    0       0    ┐
Rx(φ) = │ 0   cos(φ) -sin(φ)│
        └ 0   sin(φ)  cos(φ)┘

        ┌  cos(θ)  0  sin(θ)┐
Ry(θ) = │   0      1    0   │
        └ -sin(θ)  0  cos(θ)┘
```

El producto R = Ry(θ) · Rx(φ) da:

```
    ┌  cos(θ)        sin(θ)·sin(φ)     sin(θ)·cos(φ) ┐
R = │    0              cos(φ)            -sin(φ)     │
    └ -sin(θ)        cos(θ)·sin(φ)     cos(θ)·cos(φ) ┘
```

### 2.4. Compensación en tiempo real

En cada lectura de la IMU:

1. Se aplica la rotación al vector de aceleración:
   ```
   [ax', ay', az']ᵀ = R · [ax, ay, az]ᵀ
   ```

2. Se resta la gravedad del eje Z rotado:
   ```
   az' = az' - g    (donde g = 9.80665 m/s²)
   ```

3. `ax'` y `ay'` son las aceleraciones horizontales limpias de gravedad.

---

## 3. Referencias Externas

| Documento | Autor | Descripción |
|:----------|:------|:------------|
| [AN-1057: Using an Accelerometer for Inclination Sensing](https://www.analog.com/en/resources/app-notes/an-1057.html) | Analog Devices | Application note del fabricante de la ADIS16460. Define las relaciones trigonométricas entre el vector de gravedad y los ejes del acelerómetro. |
| [AN3461: Tilt Sensing Using a Three-Axis Accelerometer](https://www.nxp.com/docs/en/application-note/AN3461.pdf) | NXP / Freescale | Derivación detallada de las ecuaciones roll/pitch con `atan2`. Discute sensibilidad angular y calibración. |
| [AN3461: Tilt measurement using a low-g 3-axis accelerometer](https://www.st.com/resource/en/application_note/an3461-tilt-measurement-using-a-lowg-3axis-accelerometer-stmicroelectronics.pdf) | STMicroelectronics | Mismas fórmulas aplicadas a acelerómetros MEMS de ST. Incluye discusión de filtrado y calibración de offset. |
| [ADIS16460 Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16460.pdf) | Analog Devices | Datasheet del sensor. Define el sistema de coordenadas (mano derecha), escalas de sensibilidad, y registros de calibración. |

---

## 4. Procedimiento de Calibración

### 4.1. Requisitos previos

- El robot debe estar en su **posición de operación normal** (ruedas en el suelo, carga montada si aplica)
- El robot debe estar **completamente quieto** sobre una **superficie plana y nivelada**
- Los motores y actuadores deben estar **apagados** para evitar vibraciones

### 4.2. Ejecución

```bash
# Lanzar el nodo IMU (si no está activo)
ros2 launch autonomous_robot_localization_pkg imu_launch.py

# Ejecutar calibración de tilt
ros2 service call /imu_calibration std_srvs/srv/Trigger
```

El proceso tarda **15 segundos** (constante `kCalibrationDurationSec`). Durante este tiempo:
1. Se recopilan ~1500 muestras de aceleración (a 100 Hz)
2. Se promedian para obtener el vector de gravedad estático
3. Se calculan roll y pitch
4. Se construye y almacena la matriz de rotación
5. Se guarda la calibración en disco

### 4.3. Verificación

Tras la calibración, con el robot quieto, las lecturas deberían ser:

```bash
# Verificar que ax ≈ 0, ay ≈ 0, az ≈ 0 (gravedad eliminada)
ros2 topic echo /imu/data --field linear_acceleration
```

Valores esperados:
- `x`: ≈ 0.0 (± ruido del sensor, típicamente < 0.01 m/s²)
- `y`: ≈ 0.0 (± ruido del sensor)
- `z`: ≈ 0.0 (gravedad restada)

### 4.4. Fichero de calibración

La calibración se persiste en `~/.ros/imu_tilt_calibration.yaml` (configurable via parámetro ROS `tilt_calibration_file`).

Formato:
```yaml
tilt_calibration:
  timestamp: "2026-05-30T20:00:00Z"
  num_samples: 1500
  roll_rad: 0.0175
  pitch_rad: -0.0087
  roll_deg: 1.003
  pitch_deg: -0.498
  rotation_matrix:
    - 0.999962    # r00
    - 0.000153    # r01
    - 0.008727    # r02
    - 0.000000    # r10
    - 0.999847    # r11
    - -0.017452   # r12
    - -0.008728   # r13
    - 0.017451    # r21
    - 0.999809    # r22
```

Este fichero se carga automáticamente en cada reinicio del nodo. Para recalibrar, simplemente vuelve a llamar al servicio.

### 4.5. Cuándo recalibrar

- Si se cambia la posición de montaje de la IMU en el robot
- Si se modifica la carga o estructura del robot de forma que cambie su inclinación
- Si se observa deriva en las mediciones de posición del EKF

---

## 5. Limitaciones

1. **Solo inclinación estática**: Este método compensa la desalineación de montaje de la IMU. NO compensa inclinaciones dinámicas (rampas, terreno irregular). Para eso se necesitaría un filtro complementario o Madgwick/Mahony que estime orientación en tiempo real.

2. **Requiere calibración con el robot quieto**: Si el robot se mueve durante la calibración, los resultados serán incorrectos porque las aceleraciones dinámicas contaminarán el cálculo del vector de gravedad.

3. **Solo acelerómetro**: El giróscopo no se compensa porque para inclinaciones pequeñas (< 5°) la proyección de ωz cambia por cos(θ) ≈ 0.996, lo cual es despreciable. Además, el giróscopo no se utiliza actualmente en el pipeline de procesamiento.

4. **Asume gravedad estándar**: Se usa `g = 9.80665 m/s²` (constante definida en `ADIS16460_driver.hpp`). La variación de gravedad con la latitud y altitud es despreciable para este uso.

---

## 6. Impacto en el EKF

El EKF en `ekf_node.cpp` usa `linear_acceleration.x` y `linear_acceleration.y` directamente como entradas de control:

```cpp
predict(msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        dt);
```

Con la compensación de tilt activa:
- `linear_acceleration.x` ≈ aceleración real del robot en el eje frontal (sin componente de gravedad)
- `linear_acceleration.y` ≈ aceleración real del robot en el eje lateral (sin componente de gravedad)

**No se requieren cambios en el EKF.** La compensación es transparente: el nodo IMU publica datos ya corregidos.
