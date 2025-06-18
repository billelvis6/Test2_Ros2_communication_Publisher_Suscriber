 projet ROS2 avec `SensorPublisher

---

```markdown
# 📡 ROS2 Sensor Publisher

## 🧾 Description

Ce projet ROS2 en C++ implémente un **nœud éditeur** (`sensor_publisher`) qui simule un capteur environnemental en publiant régulièrement des données de **température**, **humidité**, et **pression atmosphérique** sur un **topic ROS2 `/sensor_data`**.

Les données sont générées aléatoirement dans des plages réalistes, puis envoyées sous forme de chaîne de caractères (type `std_msgs::msg::String`) à d’éventuels nœuds abonnés (subscribers).

---

## 🛠️ Fonctionnalités

- Simulation de capteurs environnementaux.
- Publication de données toutes les **500 ms**.
- Utilisation d’un **Timer ROS2** pour la périodicité.
- Affichage en temps réel des valeurs publiées.
- Utilisation de ROS2 avec **C++ et rclcpp**.

---

## 📦 Architecture

```

sensor\_data\_evaluation/
├── src/
│   └── sensor\_publisher.cpp       # Code source principal
├── CMakeLists.txt                 # Configuration CMake
├── package.xml                    # Métadonnées du package ROS2
└── README.md                      # Ce fichier

````

---

## 🚀 Exécution

### 1. Compilation
Assurez-vous d’être dans votre espace de travail ROS2 :
```bash
cd ~/ros2_ws
colcon build --packages-select sensor_data_evaluation
source install/setup.bash
````

### 2. Lancer le Publisher

```bash
ros2 run sensor_data_evaluation sensor_publisher
```

> Assurez-vous que `ros2 run` est lancé après `source install/setup.bash`.

---

## 📥 Données publiées

Les messages sont publiés sur le topic :

```
/sensor_data   [std_msgs/msg/String]
```

### Exemple de message publié :

```
"28.5,65.2,1002.7"
```

Cela correspond à :

* Température : 28.5 °C
* Humidité : 65.2 %
* Pression : 1002.7 hPa

---

## 🔧 Détails techniques

### Génération des données aléatoires :

* **Température** : entre **15°C** et **35°C**
* **Humidité** : entre **30%** et **70%**
* **Pression** : entre **950** et **1050 hPa**

Générées avec :

```cpp
float temperature = 15 + static_cast<float>(std::rand()) / (static_cast<float>(RAND_MAX / (35 - 15)));
```

---

## 📚 Dépendances

* ROS2 Humble (ou autre distribution ROS2 compatible)
* `rclcpp`
* `std_msgs`

---

## 📌 À faire

* Ajouter un Subscriber ROS2 pour lire les données publiées.
* Ajouter une interface graphique ou des visualisations avec `rqt_plot` ou RViz.
* Simuler des erreurs ou des capteurs multiples.

---

## 👨‍💻 Auteur

Projet développé dans le cadre de l’apprentissage de ROS2 et du langage C++ par **Bill-Elvis Somakou**.


