
# 🚀 ROS2 Project – Communication Publisher/Subscriber

Ce projet met en œuvre un système de communication de type **Publisher/Subscriber** sous **ROS2** utilisant le langage **C++**.

---

## 📌 Objectif du projet

Créer un nœud ROS2 **Publisher** qui simule des données de capteurs (Température, Humidité, Pression) et les publie sur un topic appelé `/sensor_data`.  
Un nœud **Subscriber** s'abonne à ce topic, reçoit les données et les affiche.

---

## 🧱 Architecture

- **Publisher** : génère aléatoirement les valeurs de température, humidité, pression.
- **Subscriber** : reçoit les données, les convertit, puis les affiche proprement.

---

## 📦 Dossier

```

ros2\_ws/
├── src/
│   └── sensor\_data\_evaluation/
│       ├── src/
│       │   ├── sensor\_publisher.cpp
│       │   └── sensor\_subscriber.cpp
│       ├── CMakeLists.txt
│       └── package.xml
└── README.md

````

---

## 🔧 Compilation et exécution

### 1. Compiler

```bash
cd ~/ros2_ws
colcon build --packages-select sensor_data_evaluation
source install/setup.bash
````

### 2. Exécuter les nœuds

Ouvrir deux terminaux :

**Terminal 1 : Subscriber**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run sensor_data_evaluation sensor_subscriber
```

**Terminal 2 : Publisher**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run sensor_data_evaluation sensor_publisher
```

---

## 💡 Résultat attendu

Affichage de ce type dans le terminal du subscriber :

```
Temp: 28.1°C | Humidity: 54.2% | Pressure: 1002.1 hPa
Temp: 25.9°C | Humidity: 60.1% | Pressure: 1010.7 hPa
...
```



## 📸 Exemple de simulation (si tu ajoutes une image plus tard)

![Simulation ROS2](./img/simulation.png)

---

## ✍️ Auteur

**Bill Elvis Somakou**
Spécialité Systèmes Embarqués – ROS2 & C++


