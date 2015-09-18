**Table of Contents**  *generated with [DocToc](http://doctoc.herokuapp.com/)*

- [Installation des ROS-Systems Version Indigo für Ubuntu 14.04 LTS](#)
- [Einrichten des Workspace](#)
- [Download und Installation des SICK LD-MRS Treibers](#)
- [Download und Installation des IMU Plugins für Rviz](#)
- [Download und Installation der Projekt-Pakete](#)
- [Pfade der Multicar Konfiguration anpassen](#)
- [Zusätzliche Pakete installieren](#)
- [Messung starten](#)
- [Aufzeichnung und Wiedergabe von Messungen](#)

####Installation des ROS-Systems Version Indigo für Ubuntu 14.04 LTS

*Anweisungen folgen und dabei die Variante Desktop-Full Install wählen*

http://wiki.ros.org/jade/Installation/Ubuntu

Zusätzlich sind noch folgende ROS Packages notwendig:

`sudo apt-get install ros-jade-geographic-msgs ros-jade-geodesy ros-jade-moveit-core ros-jade-robot-localization`


####Einrichten des Workspace

*Anweisungen folgen und den Workspace einrichten*

http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Hinweis: Falls anaconda Python Distribution installiert ist, gibt es Konflikte mit dem `catkin_pkg`, was nicht gefunden wird. Einfachste Möglichkeit: In der `~/.bashrc` einfach die Zeile `export PATH="/home/pbalzer/anaconda/bin:$PATH"` auskommentieren und Terminal neu starten. Dann wird das System Python genutzt und `catkin_make` funktioniert.

Die nun erstellte `setup.bash` in `catkin_ws/devel/` sollte noch in die `~/.bashrc` aufgenommen werden.
`source /home/pbalzer/catkin_ws/devel/setup.bash`

####Download und Installation des SICK LD-MRS Treibers

*Anlegen eines Download-Verzeichnisses*

`mkdir sick`

`cd sick`

*Download des Projektes*

(`sudo apt-get install bzr`)

`bzr branch http://bazaar.launchpad.net/~csiro-asl/csiro-asl-ros-pkg/electric`

*Umbenennen und kopieren der Dateien in den Workspace*

`mv electric csiro-asl-ros-pkg`

`cp csiro-asl-ros-pkg -r ~/catkin_ws/src`

*Kompilieren des Programms im Workspace*

`catkin_make`

*Evtl. muss die Python-Node ausführbar gesetzt werden, damit ROS sie per rosrun findet*

`chmod +x ~/catkin_ws/src/csiro-asl-ros-pkg/sick_ldmrs/nodes/sickldmrs.py`

*Möchte man den Namen des Topics für die Pointcloud2 ändern, so kann dies in der Datei csiro-asl-ros-pkg/sick_ldmrs/nodes/sickldmrs.py erfolgen. Entweder passt man folgende Zeichenkette dauerhaft an:*

`topics['cloud'] = rospy.Publisher('/%s/cloud' % node_name,
                                      numpy_msg(PointCloud2),queue_size=1000)`
                                      
*oder man ändert nur den zweiten Teil des Topic-Namen „cloud“ in der Zeile:*

`topics = {}.fromkeys(( 'cloud',  'scan0',  'scan1',  'scan2',  'scan3'))`

*Im zweiten Fall lautet der Topic-Name dann „/sickldmrs/xxx“*

####Download und Installation des IMU Plugins für Rviz

`git clone https://github.com/ccny-ros-pkg/imu_tools.git`

*Verschieben in den Workspace und kompilieren*

`cp imu_tools -r ~/catkin_ws/src`

`cd ~/catkin_ws`

`catkin_make`

####Download und Installation der Projekt-Pakete

*Klonen des Git-Archives und verschieben der vier Pakete [multicar_hydraulic](https://github.com/gus484/ros/tree/master/multicar_hydraulic), [tinkerforge_laser_transform](https://github.com/gus484/ros/tree/master/tinkerforge_laser_transform), [tinycan](https://github.com/gus484/ros/tree/master/tinycan) und [Multicar_moveit_config](https://github.com/gus484/ros/tree/master/Multicar_moveit_config) in den Workspace*

`git clone https://github.com/gus484/ros.git`

`cd ros`

`cp  multicar_hydraulic tinkerforge_laser_transform tinycan Multicar_moveit_config ~/catkin_ws/src -R`

####Installation von Octomap Server

`sudo apt-get install ros-jade-octomap-msgs ros-jade-octomap-ros`

Für Jade ist Octomap Server noch nicht als Paket verfügbar. Da nehmen wir einfach das für Indigo von Github:

`git clone https://github.com/OctoMap/octomap_mapping.git`
`cp octomap_mapping/ -r ~/catkin_ws/src/`

*Workspace kompilieren*

`cd ~/catkin_ws`

`catkin_make`

Hinweis: Wenn Fehler auftreten, einfach nochmal `catkin_make` ausführen. Durch die Parallelisierung kann es passieren, dass Dateien, die zum Kompilieren notwendig sind, vom anderen Core noch nicht berechnet wurden und dann stoppt es mit Fehler.

![Multithreading Dogs](https://what.thedailywtf.com/uploads/default/11505/6f3925bd03870f21.jpg)

####Pfade der Multicar Konfiguration anpassen

*URDF-Datei und .dae-Datein aus Git Repository holen, Arbeitsordner anlegen und Dateien verschieben:*

`mkdir ~/ROS_MoveIt`

`cp ROS_MoveIt/*.dae ROS_MoveIt/multicar.urdf ~/ROS_MoveIt`

*Pfade in URDF-Datei anpassen*

`nano ~/ROS_MoveIt/multicar.urdf`

*URDF Pfad im MoveIt Paket in folgenden Dateien anpassen:*

`nano ~/catkin_ws/src/Multicar_moveit_config/.setup_assistant`

`nano ~/catkin_ws/src/Multicar_moveit_config/launch/planning_context.launch`

*Pfade in Launchfiles anpassen:*

`nano ~/catkin_ws/src/tinkerforge_laser_transform/launch`


####Messung starten

*roscore starten*

`roscore`

*Je nach Anwendung entsprechendes launch file aus der [tinkerforge_laser_transform Node](https://github.com/gus484/ros/tree/master/tinkerforge_laser_transform/launch) im Verzeichnis launch starten. Beispiel:*

`cd ~/catkin_ws`

`roslaunch src/tinkerforge_laser_transform/launch/Messung.launch`

####Anschauen was abgeht

`rosrun rqt_topic rqt_topic` kann man aktive Topics sehen
`rosrun rqt_graph rqt_graph` kann man das visualisiert ansehen

####Aufzeichnung und Wiedergabe von Messungen

*Siehe Anleitung im Git-Repository:*

https://github.com/gus484/ros/tree/master/recorded_maps
