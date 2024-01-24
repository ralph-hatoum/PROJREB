# Projet robotique / embarqué - Robot Pepper - 5TC INSA Lyon

## Objectif

Faire bouger Pepper suivant des waypoints, détecter une boîte entre deux waypoints, diminuer la vitesse et pousser la boîte jusqu'au prochain waypoint

## Quick start Pepper

Allumer Pepper, récupérer son adresse IP et s'y connecter en ssh par : 
```
ssh nao@< addresse IP >
```

Il faut ensuite lancer ROS sur le robot, executer la commande : 
```
roslaunch pepper_bringup pepper_full.launch
````

À partir de là, on peut executer des programmes Python qui commandent le robot à travers ROS. 

## Notre code

D'abord, lancer le programme python disable_all_autonomous_behavior.py qui permet de désactiver les comportements par défaut de Pepper. Le programme se termine sur une erreur et c'est normal, nous n'avons pas réussi à la résoudre. Attention, à partir de là, le robot n'évite plus les obstacles, et monter le port de chargement ne l'empêche pas de bouger. Il faut faire attention.
Le script stop.py permet d'envoyer des messages d'arrêt au robot. Si cela ne fonctionne pas, on peut mettre le robot en veille en mettant la main sur son visage.

Le code que nous avons réalisé permet de donner une liste de waypoints dans la variable targets définie dans la fonction listener. 

Le point (0,0) correspond à la position à laquelle Pepper se trouve au moment de l'allumer. 

L'odométrie ne fonctionne pas bien, ainsi le robot ne se dépalce pas de manière très fiable. Mais il se déplace quand même.

Le prorgamme head_center.py permet de bouger la tête du robot vers le bas. Attention, comme le programme disable_all_autonomous_behavior.py n'abouti pas, le robot track les visages et si il est fixé sur un visage,  il baissera la tête puis la remontera vers le visage. Pour éviter ça, toujours relancer le disable_all_autonomous_behavior.py avant de lancer head_center.py.

Sur son chemin, le robot attend une boîte. Si il la voit, il baissera doucement sa vitesse et la poussera jusqu'au waypoint suivant.

### Quick start
Il faut lancer le programme demo_robot.py sur Pepper

### Explications pour la reconnaissance de la boîte

On utilise des codes AR qu'on place sur la boîte. 

## Pour coder efficacement avec Pepper
Quelques points à noter : 
- Sur Pepper, c'est Python 2.7 qui tourne, donc si vous codez en Python 3 il y aura des modifications à faire.
- La connexion SSH a tendance à ramer au bout d'un certain temps. Il est possible de se connecter directement en ethernet au robot, faites le c'est mieux.

Écrivez vos scripts sur vos machines personnelles, et copiez les sur le robot à l'aide de SCP. Par exemple, si votre script s'appelle script.py, faites dans le répertoir courant de votre machine :
```
scp script.py nao@< IP du robot >:.
```
Vous retrouvez en vous connectant en SSH au robot votre script. On déconseille de trop travailler directement sur le robot pour éditer les scripts, de toute façon il n'y a que VIM donc pas le plus pratique.