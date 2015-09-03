# Launch-Files

Für den Messvorgang existieren verschiedene Launch-Files, welches jeweils noch konfiguriert werden können.

## Starten eines Launch-Files

* `roscore` (erstes Terminal)
* `roslaunch FILENAME

### Messung.launch
* `roslaunch /home/carpc/catkin_ws/src/tinkerforge_laser_transform/launch/Messung.launch`
* ohne Bluetooth GPS (Serial)
* ohne Rviz (auskommentiert)

## Messung_BT_GPS.launch

* `roslaunch /home/carpc/catkin_ws/src/tinkerforge_laser_transform/launch/Messung_BT_GPS.launch`
* mit Bluetooth GPS (Serial)
* mit Rviz
