# ROS2-Kurwa 🚀🤖

*A glorious colab between master coder Daniel 🧠 and his loyal sidekick Marcin 💻 (who googles a lot but tries hard)*

In this legendary project, we bravely navigate the strange and wonderful world of **ROS2 Jazzy** 🪩 to build a robot joint controller with custom services, URDF magic, and launch files that may or may not work on the first try.

## 🔧 Technologies used

- ROS2 Jazzy Fitzroy 🦘
- `ament_cmake` (because why make it easy?)
- Custom ROS2 service messages (yes, we made our own like pros)
- URDF (aka XML yoga for robots)
- Launch files (aka: "Why isn’t my node starting?")
- Git (most of the time correctly)
- Friendship and frustration in equal parts ❤️🔥

## 🧩 Project structure

This project is split between two brave adventurers:

### 🧑‍💻 Daniel the Wizard – Handles:
- Writing URDF files like it's a sacred scroll
- Creating the **Service Server**
- Pretending the math is easy

### 🧑‍🚀 Marcin the Button Presser – Handles:
- Making **Launch files** that hopefully launch something
- Writing the **Service Client**
- Saying "it works on my machine"

## 📦 Packages

### `pid_controller_msgs`

Custom service package that contains our mighty `.srv` file:

```srv
float64 request
---
bool success
