
## 📄 `sensor_subscriber` – ROS2 C++ Node

### 🎯 Objectif

Ce nœud ROS2 nommé `sensor_subscriber` est conçu pour **écouter** (s'abonner à) un topic nommé `/sensor_data` dans lequel un autre nœud (le `publisher`) publie des données simulées de **température**, **humidité**, et **pression atmosphérique**.

Il reçoit les données, les **extrait**, les **affiche dans le terminal**, et **vérifie si les valeurs sont dans les plages normales**.

---

### 📌 Fonctionnement général

* Le code définit une classe `SensorSubscriber` qui **hérite de `rclcpp::Node`**, ce qui signifie que c'est un nœud ROS.
* Ce nœud est **abonné** au topic `/sensor_data` et attend les messages publiés.
* Lorsqu’un message arrive, il est traité dans la méthode `process_message()` :

  * Le message est affiché brut.
  * Il est analysé (découpé) pour extraire les 3 valeurs.
  * Ces valeurs sont affichées proprement.
  * Des alertes sont générées si une valeur dépasse une plage normale.

---

### 🧠 Explication du code

```cpp
SensorSubscriber() : Node("sensor_subscriber")
```

> Constructeur de la classe. Le nœud est nommé `sensor_subscriber`.

```cpp
subscription_ = this->create_subscription<std_msgs::msg::String>(
  "/sensor_data", 10, std::bind(&SensorSubscriber::process_message, this, std::placeholders::_1)
);
```

> On crée un abonnement au topic `/sensor_data` avec une file de 10 messages maximum.
> Quand un message est reçu, on appelle `process_message()` pour le traiter.

---

### 🧪 Traitement des données

```cpp
void process_message(const std_msgs::msg::String::SharedPtr msg)
```

> Fonction appelée à chaque message reçu. Elle fait le traitement.

```cpp
std::stringstream ss(msg->data);
float temp, hum, press;
char comma1, comma2;
```

> On transforme la chaîne de caractères en flux pour extraire les 3 valeurs.

```cpp
if (!(ss >> temp >> comma1 >> hum >> comma2 >> press)) {
  RCLCPP_ERROR(this->get_logger(), "⚠️ Format de message invalide !");
  return;
}
```

> On vérifie que le format est correct, sinon on affiche une erreur.

```cpp
RCLCPP_INFO(this->get_logger(), "Température : %.2f°C | Humidité : %.2f%% | Pression : %.2f hPa", temp, hum, press);
```

> Affichage structuré dans le terminal via les logs ROS.

---

### ⚠️ Gestion des plages

```cpp
if (temp < 15 || temp > 35)
  RCLCPP_WARN(this->get_logger(), "⚠️ Température hors plage !");
```

Même chose pour humidité et pression. Cela permet de vérifier la **fiabilité des capteurs** (ou des données reçues).

---

### ✅ Améliorations ajoutées

* ✅ **Séparation du traitement** dans une fonction dédiée.
* ✅ **Logs ROS2** (INFO, ERROR, WARN) pour un meilleur suivi.
* ✅ **Sécurité** : vérification du format reçu.

---

### 🔁 À utiliser avec un Publisher

Le nœud `sensor_subscriber` doit être lancé **en parallèle** avec un publisher qui envoie des données sur `/sensor_data`.

---

### ▶️ Lancer le Subscriber

Dans un terminal :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run sensor_data_evaluation sensor_subscriber
```

---

Souhaites-tu que je te génère un `launch file` pour lancer `publisher` et `subscriber` en même temps ?
