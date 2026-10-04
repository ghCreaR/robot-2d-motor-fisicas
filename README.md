# robot-2d-motor-fisicas

Motor de simulación 2D de la [carrera de robots autónomos](https://github.com/ojgarciab/carrera-robots-autonomos).

> **Estado:** en diseño. Todavía no hay código; el plan de implementación está en [`Plan.md`](Plan.md).

## Qué es

Cada instancia de este componente es un **mundo**: un circuito con sus robots. Es la **única fuente de verdad** de lo que pasa en él. Avanza la física a paso fijo, calcula lo que "ven" los sensores de cada robot, aplica las consignas de los motores que mandan los clientes y decide cuándo entra y sale cada robot.

Solo hace cálculo. **No habla HTTP ni WebSocket y no accede a la base de datos.** Se comunica únicamente por el **bus de mensajes (NATS)**, en el que se identifica con el UUID y el testigo de su mundo. Así su bucle no se ve afectado por clientes lentos ni por picos de conexiones. Para tener varios mundos se arrancan varias instancias, cada una con su circuito.

```
                         NATS: mundo.<uuid>.…
  pasarela ── actuadores, actividad, control ──►┌────────────────────┐
           ◄── sensores, estado, eventos, ──────│  MOTOR (un mundo)  │
               latido, configuración            │  - física 2D       │
                                                │  - sensores IR     │
                                                │  - ciclo de vida   │
                                                └────────────────────┘
```

## Responsabilidades

| Área | Qué hace |
|------|----------|
| **Física** | Bucle a paso fijo (60 Hz por defecto) con un **modelo dinámico**: cada robot tiene masa e inercia, y sus ruedas lo empujan con una fuerza limitada por su adherencia. Los robots **se empujan** entre sí, y al llegar a los bordes del mapa, que son **paredes**, **rebotan**, se **arrastran** o **giran** según el ángulo, la velocidad y el rozamiento. |
| **Motores** | Aplica cada consigna en el siguiente paso de física. La velocidad real de cada rueda se acerca a la pedida respetando `aceleracion_max` y `deceleracion_max` (inercia). Con los motores desactivados, la rueda frena con `deceleracion_reposo`. |
| **Sensores** | Muestrea cada sensor a su frecuencia (10 Hz los infrarrojos), comprobando si su punto de medida cae sobre la línea del circuito (25 mm de ancho). De momento son **digitales** (`0` o `1`); más adelante habrá un sensor que promedia varias lecturas. Cada muestra lleva una marca de tiempo en milisegundos. |
| **Circuito** | Usa el circuito (óvalo, ocho…) que le indica `CIRCUITO`, cuya definición recibe de la pasarela. |
| **Ciclo de vida** | Entrada de robots en un punto libre y aleatorio, orientados hacia el centro, respetando una distancia de seguridad. Desconecta los motores sin instrucciones (5 s en polling, o al cerrarse el WebSocket) y saca el robot del mundo tras 5 minutos sin actividad o por salida voluntaria. |
| **Límite de robots** | Rechaza las entradas cuando ya hay `MAX_ROBOTS` robots (4 por defecto). |
| **Publicación** | Sensores de cada robot, estado completo del mundo para la vista de administrador (30 Hz), eventos de entrada y salida, y un latido cada `LATIDO_S` segundos. |
| **Configuración** | Al arrancar pide a la pasarela las definiciones de sus robots permitidos y de su circuito, así que no necesita una copia de los directorios `robots/` y `circuitos/`. |

## Qué no hace

- **No autentica clientes ni comprueba tokens.** Eso lo hace la pasarela; el motor confía en lo que llega por los temas de su mundo.
- **No sabe de usuarios ni de permisos.** Identifica cada robot por el UUID de su dueño.
- **No guarda nada en disco.** Si se reinicia, el mundo empieza vacío.

## Configuración

Variables de entorno (ver el [`compose.yaml`](https://github.com/ojgarciab/carrera-robots-autonomos/blob/master/compose.yaml) del repositorio común):

| Variable | Por defecto | Descripción |
|----------|-------------|-------------|
| `NATS_URL` | — | Dirección del bus. |
| `MUNDO_UUID`, `MUNDO_TESTIGO` | — | Identidad del mundo; se obtienen al darlo de alta en la interfaz de gestión. |
| `CIRCUITO` | — | Circuito del mundo: `ovalo`, `ocho`… |
| `MAX_ROBOTS` | `4` | Robots simultáneos como máximo. |
| `PASO_FISICA_HZ` | `60` | Frecuencia del paso fijo de la física. |
| `ESTADO_MUNDO_HZ` | `30` | Frecuencia del estado para la vista de administrador. |
| `LATIDO_S` | `2` | Segundos entre latidos. |
| `PARADA_MOTORES_S` | `5` | Segundos sin instrucciones antes de desconectar los motores (polling). |
| `SALIDA_MUNDO_S` | `300` | Segundos sin actividad antes de sacar el robot del mundo. |

No expone puertos: solo se conecta al bus.

## Documentación relacionada

- [Arquitectura del servidor](https://github.com/ojgarciab/carrera-robots-autonomos#arquitectura-del-servidor) y [mensajes entre componentes](https://github.com/ojgarciab/carrera-robots-autonomos#mensajes-entre-componentes)
- [Modelos de robot](https://github.com/ojgarciab/carrera-robots-autonomos#modelos-de-robot) y [formato de los YAML](https://github.com/ojgarciab/carrera-robots-autonomos/blob/master/robots/README.md)
- [Ciclo de vida del robot en el mundo](https://github.com/ojgarciab/carrera-robots-autonomos#ciclo-de-vida-del-robot-en-el-mundo)
- [Contrato de los mensajes del bus](https://github.com/ojgarciab/carrera-robots-autonomos/blob/master/contratos/bus.md) y [formato de los circuitos](https://github.com/ojgarciab/carrera-robots-autonomos/blob/master/circuitos/README.md)
- [Plan de implementación](Plan.md)

## Licencia

[GPL-3.0](LICENSE).
