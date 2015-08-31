# Aufzeichnen und Wiedergeben von Messungen

Zur Aufzeichnung und Wiedergabe von Messfahrten kann das Tool rosbag eingesetzt werden. Nachfolgend wird kurz der Ablauf eines solchen Szenarios beschrieben. Die Verwendung von rosbag und mögliche Parameter können auf der ROS-Wiki-Seite [rosbag](http://wiki.ros.org/rosbag) eingesehen werden.

## Aufzeichnen einer Messung

* starten des Systems (Sensoren, Octomap, ...) mittels roslaunch
* `rosbag record --all --output-name=NAME`

## Filtern einer Messung
* `rosbag filter INBAG OUTBAG EXPRESSION`
 * EXPRESSION:  "topic=='/imu/data' or topic== '/cloud' or topic== '/fix' or topic== '/odom'"


## Wiedergabe einer Messung

* Vorbereitungen
 * roscore starten
 * `rosparam set use_sim_time true` (Simulationszeit im Rviz verwenden)
 * vorbereitetes launch file mit roslaunch starten
* Wiedergabe
 * `rosbag play FILE --clock -r SPEED`
