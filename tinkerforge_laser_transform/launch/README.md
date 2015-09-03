## Launch-Files

Für den Messvorgang existieren verschiedene Launch-Files, welche jeweils noch konfiguriert werden können. Soll der Debug-Modus der Node robot_localization genutzt werden, so muss der Speicherpfad des Parameters "debug_out_file" angepasst werden. Ebenso bei Einsatz von Rviz der Pfad zum Urdf-File des Parameters "robot_description". Soll das Tinkerforge GPS-Bricklet eingesetzt werden, ist der Parameter "gps_msgs" der Node tinkerforge_laser_transform auf true zu setzen.

### Starten eines Launch-Files

* `roscore` (erstes Terminal)
* `roslaunch FILENAME` (zweites Terminal)

### Messung.launch
* `roslaunch /home/carpc/catkin_ws/src/tinkerforge_laser_transform/launch/Messung.launch`
* ohne Bluetooth GPS (Serial)
* ohne Rviz (auskommentiert)

### Messung_BT_GPS.launch

* `roslaunch /home/carpc/catkin_ws/src/tinkerforge_laser_transform/launch/Messung_BT_GPS.launch`
* mit Bluetooth GPS (Serial)
* mit Rviz

### Wiedergabe.launch

* `roslaunch /home/carpc/catkin_ws/src/tinkerforge_laser_transform/launch/Wiedergabe.launch`
* angepasst zur Wiedergabe mittels rosbag
* ohne GPS
* mit Rviz
