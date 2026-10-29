# Latencia de la nube de puntos de la RealSense en el mapa de elevación

Diagnóstico y solución del mapa de elevación "lentísimo" con la D435i del mapa
(`elevationmap_camera_01`, serie `141722076895`) en el robot real. Fecha: 2026-10-29.

Es el mismo tipo de problema que `PCL_LATENCY_REPORT.md` (Isaac Sim, Fast DDS), aquí con CycloneDDS.

## Síntoma

El mapa de elevación se actualizaba aproximadamente una vez por segundo.
`elevation_mapping_node` publicaba a su frecuencia configurada (`elevation_map_raw` 5 Hz,
`elevation_map_filter` 3 Hz), pero solo cambiaba cuando llegaba una nube nueva, y llegaban ~1 Hz.

## Resultado

| | Antes | Después |
|---|---|---|
| Nubes recibidas por el mapa | ~0,9–1,5 Hz | **15 Hz** (todas las que produce la cámara) |
| Tamaño de cada nube | ~4 MB (~250 000 puntos) | ~330 KB (~20 000 puntos) |
| `depth/camera_info` | ~0,9–1,3 Hz | 15 Hz |
| QoS del publicador de la nube | `RELIABLE` | `BEST_EFFORT` |
| Pérdidas UDP por buffer (`RcvbufErrors`) | hasta ~430/s | 0 |
| Enlace USB de la cámara | USB 2.0 (480 Mb/s) | USB 3.2 (5000 Mb/s) |

`elevation_map_raw` y `elevation_map_filter` siguen a 5 Hz y 3 Hz (lo configurado en
`config/setups/menzi/base.yaml`); ahora reciben ~3 nubes nuevas entre publicaciones.

## Causas

Había cuatro cuellos de botella encadenados. Cada uno escondía al siguiente.

### 1. La cámara estaba en USB 2.0

La D435i del mapa estaba en el bus 7, detrás de tres hubs USB 2.0, compartiendo hub con un
adaptador Ethernet USB (AX88179A). Aunque estaba configurada a `640x480x15`, entregaba
~1,3 imágenes/s: hasta `camera_info`, un mensaje diminuto, llegaba a 1,3 Hz.

Cómo se vio:

```bash
cat /sys/bus/usb/devices/*/speed                    # 480 = USB 2.0, 5000 = USB 3
grep "USB type" ~/.ros/log/realsense2_camera_node_*.log   # el driver lo dice: "Device USB type: 3.2"
```

Ojo: el serial USB de `/sys/bus/usb/devices/*/serial` **no** coincide con el serial de la
RealSense. Para saber qué cámara es cuál, usa el log del driver ("Device physical port") o
los ficheros `/dev/bus/usb/<bus>/<dev>` abiertos por cada proceso.

### 2. Nubes enormes con un suscriptor `RELIABLE` (RViz)

Ya en USB 3, el driver abrió el perfil por defecto `848x480x30`. Cada nube pesaba 3–6 MB, a
30 Hz (~100–180 MB/s). La nube se publicaba en `RELIABLE` (`pointcloud.pointcloud_qos: DEFAULT`)
y RViz se suscribía también en `RELIABLE`.

Un escritor `RELIABLE` de CycloneDDS espera a que cada lector fiable confirme los fragmentos
de un mensaje de varios MB antes de aceptar el siguiente. Como el driver publica desde el
mismo hilo de frames, **todo** el driver se frenaba: la CPU del driver estaba a ~0 %, con
todos los hilos dormidos esperando, y `camera_info` bajó a <1 Hz.

`elevation_mapping_node` ya se suscribe en `BEST_EFFORT` (`sensor_data`), así que el
freno venía solo del otro suscriptor. Desactivar el display en RViz no bastó (la
suscripción seguía `RELIABLE`), por eso la solución se puso en el publicador.

### 3. Buffers UDP de 212 KB

`net.core.rmem_max` estaba en 212992 bytes (valor por defecto de Linux). Una nube de varios
MB llega en una ráfaga de fragmentos UDP que desborda ese buffer. Para un lector
`BEST_EFFORT`, perder **un** fragmento significa descartar **la nube entera**.

```bash
sysctl net.core.rmem_max
# contador de pérdidas: columna RcvbufErrors de la línea Udp:
grep -A1 '^Udp:' /proc/net/snmp
```

### 4. El driver no generaba 15 nubes/s con decimación ×2

Con 15 fps, decimación ×2 (nubes de 1,5 MB), QoS `BEST_EFFORT` y buffers grandes, ya no se
perdía nada (0 `RcvbufErrors`) y la imagen de profundidad llegaba a 15 Hz, pero el driver
solo publicaba ~9,2 nubes/s (medido con un suscriptor `raw=True`, sin deserializar). Con
decimación ×3 publica las 15.

## Solución

### Repo `Realsense` (commit `164b487`)

`pixi.toml`, task `elevationmap-camera`:

```
depth_module.depth_profile:=848x480x15
decimation_filter.enable:=true
decimation_filter.filter_magnitude:=3
config_file:=config/elevationmap_camera.yaml
```

- 15 fps en lugar de 30: el mapa publica a 5 Hz, más no aporta.
- Decimación ×3: ~20 000 puntos por nube. A 3 m los puntos quedan a 1–2 cm, mucho más
  denso que las celdas de 10 cm del mapa (6 m × 6 m = 3600 celdas).

`config/elevationmap_camera.yaml`:

```yaml
pointcloud.pointcloud_qos: SENSOR_DATA
```

Va en un fichero porque `rs_launch.py` **descarta** los argumentos de lanzamiento que no
declara, y `pointcloud.pointcloud_qos` no está en su lista. Desde `config_file` imprime un
aviso amarillo ("not supported") pero sí lo pasa al nodo. Comprobación:

```bash
ros2 topic info -v /elevationmap_camera_01/elevationmap_camera_01/depth/color/points | grep -E 'Node name|Reliability'
# el publicador elevationmap_camera_01 debe salir BEST_EFFORT
```

`pixi.toml`, `[activation.env]`:

```toml
CYCLONEDDS_URI = '<CycloneDDS><Domain><Internal><SocketReceiveBufferSize min="default" max="8MB"/></Internal></Domain></CycloneDDS>'
```

### Repo `elevation_mapping_gpu_ros2` (commit `5d674fb`)

El mismo `CYCLONEDDS_URI` en `docker/run.sh` y `docker-compose.yml`.

`min="default"` hace que CycloneDDS arranque aunque el kernel no permita 8 MB (con
`min="8MB"` fallaría al crear el dominio). Con `max="8MB"` pide 8 MB y el kernel lo limita a
`rmem_max`. El kernel duplica el valor pedido, por eso `ss -u -a -m` muestra `rb16777216`.

### Sistema (requiere `sudo`, una vez por máquina)

```bash
sudo sysctl -w net.core.rmem_max=16777216
echo 'net.core.rmem_max=16777216' | sudo tee /etc/sysctl.d/90-ros-dds.conf
```

Solo sube el máximo permitido; no cambia el buffer por defecto de otros procesos (la VIO
no se ve afectada). Los procesos ROS deben **reiniciarse** después, porque el buffer se pide
al abrir el socket.

### Hardware

La cámara del mapa conectada directamente a un puerto USB 3, con cable USB 3, sin hubs USB 2.0.

## Cómo se ejecuta

```bash
# cámara y TF (repo Realsense), igual que antes
pixi run -e humble tf-bridge
pixi run -e humble elevationmap-camera

# elevation map: recrear el contenedor una vez para que tenga CYCLONEDDS_URI
./docker/run.sh        # o docker compose up
ros2 launch elevation_mapping_cupy elevation_mapping.launch.py use_sim_time:=False launch_rviz:=false
```

En un contenedor creado antes de este cambio, exportar la variable antes de lanzar:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><Internal><SocketReceiveBufferSize min="default" max="8MB"/></Internal></Domain></CycloneDDS>'
```

Mensajes normales al arrancar la cámara:

- `Parameter 'pointcloud.pointcloud_qos' in config file is not supported`: se aplica igualmente.
- `failed to claim usb interface ... RS2_USB_STATUS_BUSY`: el driver intenta abrir la otra
  D435i (la de la VIO), que ya está en uso, y la ignora.

## Cómo comprobarlo

```bash
P=/elevationmap_camera_01/elevationmap_camera_01
ros2 topic hz $P/depth/camera_info              # ~15 Hz: la cámara entrega todos los frames
ros2 topic hz $P/depth/color/points             # ~15 Hz
ros2 topic hz /elevation_mapping_node/elevation_map_raw   # 5 Hz
sysctl net.core.rmem_max                        # 16777216
grep -A1 '^Udp:' /proc/net/snmp                 # RcvbufErrors no debe crecer
```

`ros2 topic hz` es Python y puede quedarse corto con mensajes grandes. Para contar las nubes
sin ese sesgo, suscribirse con `raw=True` (sin deserializar):

```python
import sys, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
rclpy.init(); n = rclpy.create_node('pcl_rate_probe'); c = [0]
n.create_subscription(PointCloud2, sys.argv[1], lambda m: c.__setitem__(0, c[0] + 1),
                      qos_profile_sensor_data, raw=True)
t0 = time.time()
while time.time() - t0 < 15: rclpy.spin_once(n, timeout_sec=0.1)
print(f"{c[0] / (time.time() - t0):.2f} Hz")
```

## Diagnóstico rápido si vuelve a pasar

1. ¿`camera_info` va lento? El problema está en la cámara o en el driver, no en la red:
   revisa la velocidad USB y si el driver está bloqueado por un suscriptor `RELIABLE`.
2. ¿`camera_info` va a 15 Hz pero la nube no? Mira `RcvbufErrors`: si crece, son los
   buffers UDP; si no crece, el driver no genera más nubes (reducir tamaño).
3. ¿La nube llega a 15 Hz pero el mapa no cambia? Revisa la TF `odom -> base ->
   elevationmap_camera_01_link` y los `fps` de los publicadores en `config/setups/menzi/base.yaml`.
