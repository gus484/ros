# Aufzeichnen und Wiedergeben von Messungen

Zur Aufzeichnung und Wiedergabe von Messfahrten kann das Tool rosbag eingesetzt werden. Nachfolgend wird kurz der Ablauf eines solchen Szenarios beschrieben. Die Verwendung von rosbag und mögliche Parameter können auf der ROS-Wiki-Seite [rosbag](http://wiki.ros.org/rosbag) eingesehen werden.

## Aufzeichnen einer Messung

* starten des Systems (Sensoren, Octomap, ...) mittels roslaunch
* rosbag record --all --output-name=NAME

## Wiedergabe einer Messung

* Vorbereitungen
 * roscore starten
 * rosparam set use_sim_time true
 * vorbereitetes launch file mit roslaunch starten
* Wiedergabe
 * rosbag play FILE --clock -r SPEED
