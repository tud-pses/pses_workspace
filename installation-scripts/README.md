# Installationsanweisungen
* Falls der Benutzername nicht `pses` lautet, dann die Variable `user` in den bereitgestellten Skriptdateien entsprechend ändern. 
* Skriptdateien ausführbar machen
```bash 
sudo chmod a+x install*.sh 
```
*	install1.sh ausführen `./install1.sh`

* Entwicklungsumgebung definieren:
```bash 
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
*	install2.sh ausführen `./install2.sh`

* Entwicklungsumgebung fertig definieren:
```bash 
echo "source /home/pses/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo su
echo "source /opt/ros/kinetic/setup.bash" >> /etc/bash.bashrc
echo "source /home/pses/catkin_ws/devel/setup.bash" >> /etc/bash.bashrc
source  /etc/bash.bashrc
exit
```
*	install3.sh ausführen `./install3.sh`

*	Computer neustarten

**Anmerkung:** install4.sh soll nur auf dem Auto ausführt werden. Diese Datei installiert und konfiguriert den Treiber für die Kinect.
