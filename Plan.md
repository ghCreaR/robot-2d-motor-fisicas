# Plan de implementación · robot-2d-motor-fisicas

Este plan detalla cómo construir el motor de simulación descrito en el [README del repositorio común](https://github.com/ojgarciab/carrera-robots-autonomos). Todavía no hay código: es una propuesta para revisar antes de empezar.

Los mensajes del bus están definidos en [`contratos/bus.md`](https://github.com/ojgarciab/carrera-robots-autonomos/blob/master/contratos/bus.md) del repositorio común, y los circuitos en [`circuitos/`](https://github.com/ojgarciab/carrera-robots-autonomos/tree/master/circuitos). Este plan usa esos mismos nombres.

## 1. Decisiones técnicas propuestas

| Tema | Propuesta | Motivo |
|------|-----------|--------|
| Lenguaje | **Python 3.12** con `asyncio` | Coherente con el resto de componentes. Con 4 robots a 60 Hz el cálculo es pequeño: unas pocas operaciones por robot y paso. |
| Física | **pymunk** (Chipmunk2D) para los cuerpos y los contactos, con un **modelo de rueda propio** | Los robots se empujan y chocan con paredes que los hacen rebotar, arrastrarse o girar, y eso pide un motor de cuerpos rígidos. pymunk es estable, determinista en una misma plataforma, tiene ruedas binarias para Linux y es fácil de usar desde Python. La tracción de cada rueda no la da pymunk, así que se modela aparte (sección 3.1). |
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
│   ├── fisica.py          # espacio de pymunk: cuerpos, paredes y subpasos
│   ├── ruedas.py          # modelo de motor y de rueda: tracción, adherencia, patinaje
│   ├── sensores.py        # muestreo de sensores (infrarrojo)
│   ├── circuito.py        # geometría del circuito y distancia a la línea
│   ├── materiales.py      # rozamiento y restitución de robots y paredes
│   ├── aparicion.py       # punto libre aleatorio y orientación hacia el centro
│   ├── ciclo_vida.py      # entrar, salir, expulsar, parada de motores, salida por inactividad
│   ├── modelos.py         # carga y validación de las definiciones de robot
│   └── bus/               # transporte (NATS / memoria) y mensajes
└── tests/
```

`mundo.py` y todo lo que hay debajo **no conocen el bus ni el reloj real**: reciben el tiempo y los mensajes como argumentos. Así la simulación es determinista y se puede probar paso a paso, y el reloj real y NATS solo aparecen en `bucle.py` y `bus/`.

## 3. Modelo físico

### 3.1. Modelo dinámico

Los robots se **empujan entre sí** y **rebotan o se arrastran por las paredes**, así que hace falta un modelo con masas, fuerzas y contactos. Se usa **pymunk** (Chipmunk2D) para el cuerpo rígido de cada robot y los contactos, y un **modelo de rueda propio** encima.

**Cuerpo de cada robot**

- Un `pymunk.Body` dinámico con la `masa` del YAML y su `momento_inercia` (o `masa × radio_colision² / 2` si no se indica).
- Una forma circular de radio `radio_colision`, con `friction = rozamiento` y `elasticity = restitucion`. Pymunk combina los coeficientes de los dos cuerpos en contacto multiplicándolos; se documentará así para quien ajuste los valores.
- El rozamiento con el suelo lo aportan solo las ruedas (abajo). La rueda de bola apenas frena y no se resiste a deslizar de lado.

**Paredes**

- Cuatro segmentos estáticos en los bordes del mapa (`dimensiones`), con el `rozamiento` y la `restitucion` de `paredes` del circuito.
- El rebote, el arrastre a lo largo de la pared y el giro por el roce salen solos de la resolución de contactos de pymunk: el rozamiento en el punto de contacto, que está separado del centro de gravedad, produce un par que hace girar al robot.

**Modelo de cada rueda motriz**, aplicado en cada subpaso antes de `space.step()`:

1. **Velocidad del motor.** Se acerca a la objetivo (`consigna × velocidad_max`) como mucho `aceleracion_max × dt` si debe acelerar, o `deceleracion_max × dt` si debe frenar. Es el estado interno del motor, con su inercia.
2. **Carga sobre la rueda.** Se reparte el peso (`masa × 9,81`) entre las ruedas y los `apoyos` con el equilibrio estático según sus posiciones en `x`. En los robots de prácticas, con las ruedas en `x = 0,020` y la bola en `x = −0,050`, cada rueda soporta el 35,7 % del peso.
3. **Fuerza máxima** de la rueda: `adherencia × carga`.
4. **Impulso longitudinal.** Se calcula la velocidad del suelo bajo la rueda en la dirección de avance (`v + ω × r`) y el impulso necesario para igualarla a la del motor, con la masa efectiva del cuerpo en ese punto y esa dirección: `1 / (1/m + (r × n)² / I)`. Se limita a la fuerza máxima por `dt`. Si se alcanza el límite, la rueda **patina**.
5. **Impulso lateral.** Igual, pero para anular la velocidad lateral de la rueda, que es lo que impide que el robot se deslice de lado. También está limitado: si otro robot empuja de lado con más fuerza, el robot se desliza.
6. **Motores desactivados.** La rueda gira libre: el impulso longitudinal se limita a lo que da `deceleracion_reposo`, de modo que el robot va frenando suave. El lateral se mantiene, porque la rueda sigue apoyada.

Mientras no hay choques, los impulsos nunca llegan al límite (para acelerar a 0,5 m/s² un robot de 0,25 kg hacen falta 0,125 N, y cada rueda admite unos 0,70 N), así que el movimiento coincide con el de un modelo cinemático de tracción diferencial. Esta coincidencia es una de las pruebas de la fase 1.

**Pasos internos.** Cada paso de física (60 Hz) se divide en 4 subpasos (240 Hz) para que los contactos sean estables y los choques no atraviesen a ningún robot. Con 4 robots sigue siendo un cálculo pequeño.

Ángulos en radianes y antihorarios, igual que en el formato de los robots; `θ = 0` apunta a `+x` del mundo. La pose que se publica es la del centro de gravedad.

### 3.2. Choques entre robots y con las paredes

- **Entre robots:** los resuelve pymunk como choques entre cuerpos con masa. Como los robots de prácticas tienen la **misma masa**, ninguno tiene ventaja: si dos robots chocan de frente con la misma velocidad, se frenan; si uno está parado, el otro lo empuja y las ruedas del parado patinan según su adherencia.
- **Con las paredes:** un robot que llega de frente rebota según la restitución; si llega con un ángulo pequeño, se arrastra a lo largo de la pared y gira por el rozamiento.
- **Sensores durante un choque:** siguen funcionando igual; el robot sigue viendo la línea si pasa por encima mientras lo empujan.
- **Entrada al mundo:** la distancia de seguridad evita que un robot aparezca encima de otro o pegado a una pared.

### 3.3. Sensores infrarrojos

- El punto de medida de cada sensor se pasa a coordenadas del mundo con la pose del robot.
- Se calcula su **distancia a la línea** (al trazado más cercano del circuito). Si es menor que la mitad del ancho de la línea (12,5 mm), el valor es `1`; si no, `0`. Es el sensor **digital** `infrarrojo`, el único de la primera versión.
- **Sensor promediado (fase posterior):** el tipo `infrarrojo_promedio` toma `muestras` lecturas digitales repartidas en un disco de radio `radio_medida` alrededor del punto de medida y devuelve su promedio, de `0` a `1`. Los puntos del disco se calculan una vez al cargar el modelo (por ejemplo, en espiral de Fermat, para que queden repartidos de forma uniforme), así que el coste es `muestras` veces el del sensor digital. `sensores.py` se organiza desde el principio con una clase por tipo de sensor, para añadirlo sin tocar el resto.
- **Frecuencia:** cada sensor lleva su propio acumulador de tiempo y se muestrea cuando han pasado `1 / frecuencia_hz` segundos. A 60 Hz y 10 Hz coincide exactamente cada 6 pasos, como indica el README común. Todos los sensores de un robot que tocan en el mismo paso se publican en **un solo mensaje**, con la marca de tiempo del momento de la muestra y un número de secuencia `seq`.

### 3.4. Circuitos

Los circuitos se definen en YAML en el repositorio común, con el formato de [`circuitos/README.md`](https://github.com/ojgarciab/carrera-robots-autonomos/blob/master/circuitos/README.md): mapa en metros, ancho de línea y trazados formados por rectas y arcos. Ya están `ovalo.yaml` y `ocho.yaml`. El motor no los lleva copiados: recibe el suyo de la pasarela en la respuesta a `configuracion`.

- Con rectas y arcos la distancia a la línea es exacta: distancia a un segmento, o `|distancia al centro − radio|` si el ángulo cae dentro del arco y, si no, distancia al extremo más cercano.
- Para no recorrer todos los tramos en cada lectura, se puede usar una rejilla de celdas con los tramos que pasan por cada una. Con circuitos de 4 tramos no hace falta en la primera versión.
- El cruce del ocho sale solo de que las dos rectas se corten (a 70°).

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
- `fisica.py`, `ruedas.py`, `materiales.py` y `robot.py`.
- Pruebas de movimiento libre, comparadas con el modelo cinemático de tracción diferencial: línea recta, giro sobre sí mismo, círculo de radio conocido, tiempo de aceleración de 0 a 0,20 m/s (0,4 s con 0,5 m/s²), frenada activa frente a frenada en reposo.
- Pruebas de choques:
  - Dos robots iguales chocan de frente a la misma velocidad: se frenan y ninguno avanza.
  - Un robot empuja a otro parado: lo desplaza y las ruedas del parado patinan.
  - Choque perpendicular contra una pared: rebota según la restitución.
  - Choque rasante contra una pared: se arrastra y gira.
  - Ningún robot atraviesa a otro ni sale del mapa, ni siquiera a la velocidad máxima.
- Prueba de determinismo: la misma secuencia de consignas da la misma trayectoria.

### Fase 2 · Circuito y sensores
- `circuito.py` con rectas y arcos. Las pruebas cargan `ovalo.yaml` y `ocho.yaml` del repositorio común.
- `sensores.py` con la frecuencia por sensor.
- Pruebas: un robot parado sobre la línea ve `1` en el sensor central; los sensores contiguos no dejan pasar la línea entre ellos (separación < ancho); a 20 cm/s, cruzar la línea de frente siempre da al menos una lectura `1`.

### Fase 3 · Mundo y ciclo de vida
- `mundo.py`, `aparicion.py` y `ciclo_vida.py`, con un reloj simulado.
- Pruebas de cada fila de la tabla de la sección 4, incluidos los reintentos idempotentes, `expulsar`, `listar` y el límite de robots.

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

### Fase 7 · Mejoras posteriores
- **Sensor `infrarrojo_promedio`**: el siguiente paso previsto (sección 3.3).
- Robots veloces: más subpasos o una frecuencia de física mayor, y revisar el modelo de rueda si derrapan en las curvas.
- Ruido configurable en los sensores, para acercarse más a un robot real.
- Más tipos de sensor (distancia, encoders) a medida que aparezcan nuevos modelos de robot.

## 7. Dependencias con otros repositorios

| Depende de | Qué necesita |
|------------|--------------|
| Repositorio común | Formato de robots (con masa, rozamiento, restitución y adherencia), formato de circuitos (con las paredes; `ovalo` y `ocho` ya definidos) y contrato de mensajes (`contratos/bus.md`). |
| `robot-2d-pasarela` | *Auth callout* y respuesta a `configuracion`. Hasta entonces se prueba con el transporte en memoria. |

## 8. Decisiones tomadas

- **Sensores IR:** digitales (`0`/`1`) en la primera versión. El sensor `infrarrojo_promedio` llega en una fase posterior.
- **Circuitos:** YAML en el repositorio común; el motor los recibe de la pasarela.
- **Mensajes:** los de `contratos/bus.md`, con buzón `mundo.<uuid>.buzon` y la operación `expulsar`.
- **Choques entre robots:** se empujan. Cada robot tiene su masa; los de prácticas, la misma (0,25 kg).
- **Bordes del mapa:** son paredes. El robot rebota, se arrastra o gira según el ángulo, la velocidad y el rozamiento.
- **Motor de físicas:** pymunk, con un modelo de rueda propio que permite patinar.

## 9. Preguntas abiertas

Ninguna por ahora. Los valores de masa, rozamiento, restitución y adherencia de los YAML son una primera estimación; se ajustarán en la fase 6 viendo cómo se comportan los robots.
