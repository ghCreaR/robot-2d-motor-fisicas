# Plan de implementación · robot-2d-motor-fisicas

Este plan detalla cómo construir el motor de simulación descrito en el [README del repositorio común](https://github.com/ojgarciab/carrera-robots-autonomos). Todavía no hay código: es una propuesta para revisar antes de empezar.

Los mensajes del bus se definen en el [`Plan.md` de la pasarela](https://github.com/ghCreaR/robot-2d-pasarela/blob/main/Plan.md#33-mensajes-del-bus-messagepack) y se propone publicarlos en el repositorio común. Este plan usa esos mismos nombres.

## 1. Decisiones técnicas propuestas

| Tema | Propuesta | Motivo |
|------|-----------|--------|
| Lenguaje | **Python 3.12** con `asyncio` | Coherente con el resto de componentes. Con 4 robots a 60 Hz el cálculo es pequeño: unas pocas operaciones por robot y paso. |
| Física | **Modelo cinemático propio** de tracción diferencial | El README común lo considera suficiente para robots de 20 cm/s y evita la complejidad de un motor de físicas general. El diseño deja sitio para cambiarlo más adelante por un modelo dinámico (Box2D, Rapier…) para robots veloces. |
| Geometría | Funciones propias en Python (distancia de un punto a un segmento o a un arco) | Pocas operaciones, sin dependencias pesadas. NumPy solo si las pruebas de rendimiento lo piden. |
| Bus | **nats-py**, con transporte abstracto | Igual que la pasarela, para poder probar con un transporte en memoria. |
| Formato | **MessagePack** | Contrato común con la pasarela. |
| Pruebas | **pytest**, **hypothesis** para propiedades físicas | |
| Calidad | **ruff** y **mypy** | |

## 2. Estructura del repositorio

```
robot-2d-motor-fisicas/
├── pyproject.toml
├── Dockerfile
├── src/motor/
│   ├── __main__.py        # arranque: configuración, conexión al bus, bucle
│   ├── config.py          # variables de entorno
│   ├── bucle.py           # bucle a paso fijo y planificación de tareas periódicas
│   ├── mundo.py           # estado del mundo: robots, circuito, temporizadores
│   ├── robot.py           # pose, velocidades de rueda, consignas, estado de los motores
│   ├── cinematica.py      # integración del modelo diferencial
│   ├── sensores.py        # muestreo de sensores (infrarrojo)
│   ├── circuito.py        # geometría del circuito y distancia a la línea
│   ├── colisiones.py      # colisiones entre robots y con los bordes
│   ├── aparicion.py       # punto libre aleatorio y orientación hacia el centro
│   ├── ciclo_vida.py      # entrar, salir, expulsar, parada de motores, salida por inactividad
│   ├── modelos.py         # carga y validación de las definiciones de robot
│   └── bus/               # transporte (NATS / memoria) y mensajes
└── tests/
```

`mundo.py` y todo lo que hay debajo **no conocen el bus ni el reloj real**: reciben el tiempo y los mensajes como argumentos. Así la simulación es determinista y se puede probar paso a paso, y el reloj real y NATS solo aparecen en `bucle.py` y `bus/`.

## 3. Modelo físico

### 3.1. Cinemática diferencial

Para cada robot, en cada paso `dt = 1 / PASO_FISICA_HZ`:

1. **Velocidad de cada rueda.** Se acerca a la objetivo (`consigna × velocidad_max`) como mucho `aceleracion_max × dt` si debe acelerar, o `deceleracion_max × dt` si debe frenar. Con los motores desactivados, el objetivo es 0 y el límite es `deceleracion_reposo`.
2. **Velocidad del robot.** Con `vi` y `vd` las velocidades de las ruedas izquierda y derecha y `b` la distancia entre ellas (0,11 m en los robots de prácticas, a partir de sus `posicion`):
   - `v = (vi + vd) / 2`
   - `ω = (vd − vi) / b`
3. **Integración exacta en arco.** El robot gira alrededor del punto medio del eje de las ruedas, que no coincide con el centro de gravedad (está a `x = 0,020 m`). Se integra la pose de ese punto: si `|ω|` es casi 0, en línea recta; si no, con la fórmula del arco de circunferencia. Después se calcula la pose del centro de gravedad, que es la que se publica.

Ángulos en radianes y antihorarios, igual que en el formato de los robots; `θ = 0` apunta a `+x` del mundo.

### 3.2. Colisiones

- **Entre robots:** círculos de radio `radio_colision`. Si un paso deja dos robots solapados, se deshace el avance de ese paso en la dirección del choque y su velocidad en esa dirección pasa a 0. Es simple y estable para robots lentos.
- **Bordes del mapa:** el mismo tratamiento con los límites del circuito, para que ningún robot salga del mapa.

### 3.3. Sensores infrarrojos

- El punto de medida de cada sensor se pasa a coordenadas del mundo con la pose del robot.
- Se calcula su **distancia a la línea** (al trazado más cercano del circuito). Si es menor que la mitad del ancho de la línea (12,5 mm), el valor es `1`; si no, `0`.
- Si se decide usar un valor analógico (ver [preguntas abiertas](#8-preguntas-abiertas)), se usará la fracción de un pequeño disco de medida que cae sobre la línea.
- **Frecuencia:** cada sensor lleva su propio acumulador de tiempo y se muestrea cuando han pasado `1 / frecuencia_hz` segundos. A 60 Hz y 10 Hz coincide exactamente cada 6 pasos, como indica el README común. Todos los sensores de un robot que tocan en el mismo paso se publican en **un solo mensaje**, con la marca de tiempo del momento de la muestra y un número de secuencia `seq`.

### 3.4. Circuitos

Se propone un formato YAML en el repositorio común (`circuitos/`), igual que los robots:

```yaml
id: ovalo
nombre: Óvalo
dimensiones: [2.4, 1.6]        # m: ancho y alto del mapa, origen en una esquina
ancho_linea: 0.025             # m
trazados:                      # cada trazado es una secuencia de tramos
  - tramos:
      - {tipo: recta, desde: [0.6, 0.3], hasta: [1.8, 0.3]}
      - {tipo: arco, centro: [1.8, 0.8], radio: 0.5, inicio: -90, fin: 90}
      - {tipo: recta, desde: [1.8, 1.3], hasta: [0.6, 1.3]}
      - {tipo: arco, centro: [0.6, 0.8], radio: 0.5, inicio: 90, fin: 270}
```

Con rectas y arcos la distancia a la línea es exacta. En el ocho, el cruce sale solo de que dos tramos rectos se corten (con un ángulo de al menos 60°). Mientras se aprueba ese formato, el motor puede llevar los dos circuitos de ejemplo incluidos.

## 4. Ciclo de vida

| Situación | Comportamiento |
|-----------|----------------|
| `control` `entrar` | Si el usuario ya tiene robot, responde `recuperado` y no cambia nada. Si no, y hay sitio, lo coloca y responde `dentro`. Si está lleno, `lleno`. Es idempotente por `id_peticion` y por usuario. |
| Colocación | Punto aleatorio dentro del mapa, con margen al borde, a una distancia de seguridad de todos los robots (por ejemplo `2 × radio_colision + 0,10 m`), orientado hacia el centro del mapa. Si tras N intentos no hay hueco, responde `lleno`. |
| `actividad` `ws_abierto` | El robot pasa a modo WebSocket: los motores siguen activos mientras haya alguna conexión abierta, aunque no lleguen consignas nuevas. Lleva la cuenta de las conexiones abiertas. |
| `actividad` `ws_cerrado` | Si ya no quedan conexiones, se desactivan los motores al instante y empieza la cuenta de `SALIDA_MUNDO_S`. |
| `actuadores` | Activa los motores con la nueva consigna, que se aplica en el siguiente paso. En polling, reinicia la cuenta de `PARADA_MOTORES_S`. |
| Sin `actuadores` durante `PARADA_MOTORES_S` (polling) | Motores desactivados y evento `motores_off`. El robot frena con `deceleracion_reposo`. |
| Sin ninguna `actividad` ni `actuadores` durante `SALIDA_MUNDO_S` | El robot sale y se publica el evento `sale` con el motivo `inactividad`. |
| `control` `salir` / `expulsar` | El robot sale al instante, con su evento. La siguiente entrada es desde cero. |
| `control` `listar` | Devuelve los robots presentes, con su usuario, modelo y nombre. La pasarela lo usa para resincronizarse al arrancar. |

Las consignas que llegan entre dos pasos se guardan y **solo cuenta la última**, porque son de tipo "vale el último".

## 5. Bucle principal

- Usa `time.monotonic()` para el ritmo y `time.time()` para las marcas de tiempo que se publican.
- Los pasos se planifican por plazo absoluto (`t0 + n × dt`), no con `sleep(dt)`, para que no se acumule deriva.
- Si un paso llega tarde, se recupera hasta un máximo de pasos seguidos. Si va más retrasado, se descartan los pasos perdidos y se registra un aviso, en lugar de acelerar la simulación.
- Cada paso: aplica los mensajes recibidos, revisa los temporizadores, integra la física, resuelve las colisiones, muestrea los sensores y publica lo que toque (sensores, estado cada 2 pasos a 30 Hz, latido cada `LATIDO_S`).
- Las publicaciones no bloquean el bucle: se encolan y las envía una tarea aparte.

## 6. Fases de implementación

### Fase 0 · Esqueleto
- `pyproject.toml`, `ruff`, `mypy`, `pytest` y GitHub Actions.
- `config.py` con todas las variables y su validación.
- `Dockerfile` (`python:3.12-slim`), sin puertos expuestos y con un usuario sin privilegios.

### Fase 1 · Núcleo físico (sin bus)
- `cinematica.py` con integración exacta.
- `robot.py` con los límites de aceleración, frenada y reposo.
- Pruebas: línea recta, giro sobre sí mismo, círculo de radio conocido, tiempo de aceleración de 0 a 0,20 m/s (0,4 s con 0,5 m/s²), frenada activa frente a frenada en reposo.

### Fase 2 · Circuito y sensores
- `circuito.py` con rectas y arcos, y los circuitos `ovalo` y `ocho` de ejemplo.
- `sensores.py` con la frecuencia por sensor.
- Pruebas: un robot parado sobre la línea ve `1` en el sensor central; los sensores contiguos no dejan pasar la línea entre ellos (separación < ancho); a 20 cm/s, cruzar la línea de frente siempre da al menos una lectura `1`.

### Fase 3 · Mundo y ciclo de vida
- `mundo.py`, `aparicion.py`, `colisiones.py` y `ciclo_vida.py`, con un reloj simulado.
- Pruebas de cada fila de la tabla de la sección 4, incluidos los reintentos idempotentes y el límite de robots.

### Fase 4 · Bus
- Transporte en memoria y transporte NATS.
- Conexión con el UUID y el testigo (credenciales del *auth callout*) y prefijo de buzón `mundo.<uuid>.buzon`.
- Petición de `configuracion` al arrancar, con reintentos hasta que la pasarela responda; después carga los modelos y el circuito.
- Suscripciones a `actuadores`, `actividad` y `control`, y publicación de `sensores`, `estado`, `eventos` y `latido`.

### Fase 5 · Bucle en tiempo real
- `bucle.py` con planificación por plazo absoluto.
- Medición del tiempo de cada paso y de su retraso; aviso si supera el presupuesto.
- Prueba de rendimiento: 4 robots a 60 Hz deben usar una fracción pequeña de una CPU.

### Fase 6 · Integración
- Prueba de extremo a extremo con la pasarela y el `compose.yaml` del repositorio común: entrar, mover, leer sensores, desconectar y salir.
- Ajuste de los parámetros por defecto con los dos robots de prácticas en los dos circuitos.

### Fase 7 · Mejoras posteriores (opcionales)
- Modelo dinámico con rozamiento y derrape para robots veloces.
- Ruido configurable en los sensores, para acercarse más a un robot real.
- Más tipos de sensor (distancia, encoders) a medida que aparezcan nuevos modelos de robot.

## 7. Dependencias con otros repositorios

| Depende de | Qué necesita |
|------------|--------------|
| Repositorio común | Formato de robots (ya existe), formato de circuitos (propuesto en la sección 3.4) y contrato de mensajes. |
| `robot-2d-pasarela` | *Auth callout* y respuesta a `configuracion`. Hasta entonces se prueba con el transporte en memoria. |

## 8. Preguntas abiertas

1. **Valor de los sensores IR:** ¿digital (`0`/`1`) o analógico (`0..1`)? Se propone empezar con digital.
2. **Formato y ubicación de los circuitos:** ¿YAML en el repositorio común, como propone la sección 3.4?
3. **Choques entre robots:** ¿se bloquean como aquí, o se permite empujar a otro robot?
4. **Bordes del mapa:** ¿hay paredes, o un robot que se sale del mapa debe volver a colocarse?
