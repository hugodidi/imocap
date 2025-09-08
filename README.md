---
title: imocap – README
tags: [imocap, imu, unity, rabbitmq, gait, biomecánica, python, csharp]
---
*Se recomienda encarecidamente abrir el documento a través de Obsidian para la legibilidad y aprovechamiento de links*
# Diseño e implementación de algoritmos de captura de movimiento y procesamiento biomecánico inteligente (imocap)

Sistema modular para **captura de movimiento con IMUs**, **estimación de actitud** (cuaterniones), **streaming a Unity** y **procesamiento biomecánico** (filtros y análisis de marcha con autoencoders).

> **TL;DR**
> - `attitudecalc/`: BLE → RabbitMQ → fusión sensorial → cuaterniones → TCP a Unity  
> - `motioncapture/`: Proyectos de Unity (Lite y Humanoid, con sus respectivos Listener)
> - `ibioprocessing/`: filtros (`slidingRMS`) y `GaitAnalysis` (autoencoders + score 0–100)

---

## Tabla de contenidos

[[README.md#Estructura del repo|Estructura del repo]]  
[[README.md#Arquitectura|Arquitectura]]  
[[README.md#Requisitos|Requisitos]]  
[[README.md#Instalación|Instalación]]  
[[README.md#Configuración|Configuración]]  
[[README.md#Ejecución rápida|Ejecución rápida]]  
[[README.md#Contratos de mensajería|Contratos de mensajería]]  
[[README.md#Módulos|Módulos]]  
[[README.md#attitudecalc/|attitudecalc/]]  
[[README.md#motioncapture/|motioncapture/]]  
[[README.md#ibioprocessing/|ibioprocessing/]]  
[[README.md#GaitAnalysis (score 0–100)|GaitAnalysis (score 0–100)]]  
[[README.md#Solución de problemas|Solución de problemas]]  
[[README.md#Roadmap|Roadmap]]  
[[README.md#Licencia|Licencia]]

---

## Estructura del repo

imocap/
├─ .gitignore 
├─ README.md
├─ requirements.txt
├─ LICENSE
├─ NOTICE
├─ attitudecalc/
│  ├─ main.py                # orquestación + puente TCP a Unity
│  ├─ AttitudeEstimator.py   # fusión sensorial (cuaterniones)
│  ├─ BTClassV2_multiplex.py # BLE multiplexer → RabbitMQ
│  └─ .gitignore             # excluye pychache/ etc.
├─ ibioprocessing/
│  ├─ GaitData/gait.csv     # slidingRMS, normfunction100
│  ├─ noiseRMS_filter.py     # slidingRMS, normfunction100
│  └─ GaitAnalysisIA.ipynb   # cuaderno: autoencoders + score
└─ motioncapture/
   ├─ Body-Visualizer/Assets/ListenerUnity.cs           # Humanoid (Animator + mapeo huesos)
   ├─ IMU-Visualizer/Assets/ListenerUnity-Lite.cs      # versión simple (objeto único)
   └─ .gitignore                                                          # específico para Unity

---
## Requisitos

- **Sistema**: Windows 10/11 recomendado (BLE + RabbitMQ local).
    
- **Python**: 3.10+
    
- **Unity**: versión LTS reciente (2021+).
    
- **RabbitMQ** (y Erlang) corriendo en `localhost`.
    
- **Bluetooth LE** operativo (drivers del adaptador).

> Paquetes Python típicos (ver `requirements.txt`): `bleak`, `pika`, `numpy`, `pandas`, `keras`/`tensorflow`, `matplotlib`.

---
## Instalación

1. **Python y dependencias**

```shell
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
```

2. **RabbitMQ**
- Opción A: instalar manualmente Erlang y RabbitMQ y dejar el servicio activo.
- Opción B: `attitudecalc/main.py` contempla instalador silencioso cuando se empaqueta; en desarrollo usa A.

3. **Unity**
- Abre el proyecto de `motioncapture/` con tu versión de Unity.
- Asegúrate de tener **Input System** (o adapta a `Input.GetKeyDown`).

---
## Configuración

- **Nombres de IMU**: edita `IMUnames` en `attitudecalc/main.py` con los _device names_ reales (ej.: `["GROOT_01","GROOT_02", ...]`).
    
- **Puerto TCP**: por defecto `25001` (debe coincidir con el Listener en Unity).
    
- **Calibración**: el estimador realiza ~250 muestras en reposo al inicio. No te muevas hasta ver “Calibración finalizada”.
---
## Ejecución rápida

1. Arranca **RabbitMQ** (servicio local).
2. En **Unity**:
- Abre el proyecto adecuado:
    - **IMU-Visualizer**: contiene `ListenerUnity-Lite.cs` (control directo de un objeto sensorlike)
    - **Body-Visualizer**`ListenerUnity.cs` (Humanoid: asigna bindings `sensorName → BoneChoice`).    
> Es posible que tengas que acceder a la carpeta *Scenes* para cargar el entorno
- Pulsa **Play**.
3. En terminal:

```shell
cd attitudecalc
python main.py
```

4. **Atajos en Unity** (Es necesario tener la escena pulsada para ver e interactuar)
- **C**: recalibrar (toma la siguiente muestra como referencia).
- **L** (Humanoid): imprimir latencia media RX→Apply.
---
## Contratos de mensajería

- **Exchange de entrada (topic): `topic_BT`**  
    Payload (JSON array, 9 enteros 16-bit):  
    `[mX, mY, mZ, aX, aY, aZ, gX, gY, gZ]`  
    _El escalado (m/s², rad/s, etc.) se aplica en el estimador._
    
- **Exchange de salida (topic): `topic_Proc`**  
    Payload (JSON array, 4 float):  
    `[qw, qx, qy, qz]` (cuaternión unitario, marco global del estimador)
    
- **TCP → Unity (línea CSV por muestra):**  
    `name,qx,qy,qz,qw\n`  
    _Orden Unity (x,y,z,w); la capa Python ya ajusta reordenado/signos._
---
## Módulos

### attitudecalc/

**BTClassV2_multiplex.py**  
Escanea/Conecta por BLE a `IMUnames`, habilita notificaciones y **publica crudos** en `topic_BT` con routing key = nombre IMU.

- Métodos clave: `run()`, `stop()`.
    

**AttitudeEstimator.py**  
Suscripción a `topic_BT:<name>` → **calibración** (reposo) → **fusión sensorial** (integra gyro; corrige con acc; opcional magnetómetro) → **publica** cuaterniones en `topic_Proc:<name>`.

- Métodos clave: `run()`, `stop()`, `update()`.
    

**main.py**  
Orquesta **multiprocessing** (un proceso por IMU) y levanta un **receptor** que lee `topic_Proc:*` y envía **TCP** a Unity (formato CSV).

- Entradas a adaptar: `IMUnames`, `host`, `port`.
    

### motioncapture/

**ListenerUnity-Lite.cs**  
Servidor TCP embebido. Primera muestra fija `offset = inverse(qSensorUnity)`; aplica `transform.rotation = offset * q`.

- Tecla **C**: recalibrar.
    

**ListenerUnity.cs** (Humanoid)  
Servidor TCP + mapeo **sensor → hueso** (Bíceps/Antebrazo/Mano).  
Captura T-Pose 1 frame (reposo), calcula `M = restWorld * inverse(qSensorWorld)` por sensor, aplica `boneWorld = M * qSensorWorld` y convierte a **localRotation** usando el padre.

- Teclas: **C** (recalibrar todos), **L** (latencia media).
    
- Requiere `Animator` con rig Humanoid.
    

### ibioprocessing/

**noiseRMS_filter.py**

- `slidingRMS(window)`: filtro RMS deslizante **con signo** (suaviza ruido preservando dirección).
    
- `normfunction100(data_deg, i0, i1)`: normaliza un segmento angular a **100 puntos** (unwrap → interpola → wrap).
    

**GaitAnalysisIA.ipynb**  
Autoencoders **por articulación** (ankle/knee/hip) y cálculo de **Gait Quality Score** 0–100 por joint + clasificación global.

---
## GaitAnalysis (score 0–100)

**Dataset CSV** (mínimo): `subject, replication, leg, joint, condition, time, angle`

- `joint_map = {1: ankle, 2: knee, 3: hip}`
- Entrenamiento con `condition = 1` (marcha normativa).
- Cada seri se recorta a **100 puntos** (ordenada por `time`).

**Pipeline (resumen)**

1. Entrenar un **autoencoder por articulación** (100→64→32→16→32→64→100, ReLU; L1 en capa de 16; Adam+MSE; EarlyStopping).
2. Calcular **distribución de errores** (MSE) en entrenamiento: min p0.5, max p99.5, mean, std.
3. Inferencia: para cada joint, error de reconstrucción → **log1p** → score:  
    `score = 100 * (1 - (log_error - log_err_min) / (log_err_max - log_err_min))`, con `log_err_max = log1p(6*err_max)`.
    
4. **meanScore** = media de scores (ankle/knee/hip) → clasificación:
    
    - ≥ 75: marcha normativa
    - 50–74: pseudo-normativa
    - 25–49: alterada leve
    - < 25: alterada severa  
        _Marca articulaciones “afectadas” si su score < 40._
---
## Solución de problemas

- **“No se pudo conectar a RabbitMQ”**  
    Asegura el servicio activo en `localhost` y credenciales por defecto (si aplica).
- **“BLE: dispositivo no encontrado”**  
    Verifica que el **nombre** de la IMU en `IMUnames` coincide con el broadcast BLE; revisa permisos/driver.
- **Unity: “Connection refused”**  
    Asegura que Unity **está escuchando** (Play) y que el puerto **coincide** con `main.py`.
- **Calibración no termina**  
    Mantén las IMUs **en reposo** al inicio (~250 muestras). Repite si hay vibración continua.
- **Orientación invertida o eje incorrecto**  
    Ajusta el mapeo en `SensorToUnity` (Listener) o la conversión previa en `main.py` (signo/orden de q).
- **Latencia alta**  
    Revisa carga de CPU, desactiva trazas pesadas y mantén TCP local. El receptor usa `TCP_NODELAY`. Puntualmente, puede ayudar desactivar la calibración magnética estableciendo `self.mag_cal = False` en `AttitudeEstimator.py`
---
## Licencia

El **código** de este repositorio se distribuye bajo **MIT License** (ver `LICENSE`).

**Contenido de terceros**:
- **Dataset**: “Multivariate Gait Data” (UCI Machine Learning Repository).
  Licencia **CC BY 4.0**. Cita recomendada:
  Helwig, N. & Hsiao-Wecksler, E. (2016). *Multivariate Gait Data* [Dataset].
  UCI Machine Learning Repository. https://doi.org/10.24432/C5861T
  (ver detalles en `NOTICE`).
- **Adobe Mixamo**: personajes/animaciones usados bajo términos **royalty-free**
  de la FAQ oficial de Adobe. Atribución no obligatoria; más info en `NOTICE`.
