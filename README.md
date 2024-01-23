# Projet robotique / embarqué - Robot Pepper - 5TC INSA Lyon

## Objectif

Faire bouger Pepper suivant des waypoints, détecter une boîte entre deux waypoints, diminuer la vitesse et pousser la boîte jusqu'au prochain waypoint

## Quick start Pepper

Allumer Pepper, récupérer son adresse IP et s'y connecter en ssh par : ssh nao@< addresse IP >

Il faut ensuite lancer ROS sur le robot, executer la commande : roslaunch pepper_bringup pepper_full.launch

À partir de là, on peut executer des programmes Python qui commandent le robot à travers ROS. 