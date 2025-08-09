# ROS Colour Pub-Sub

A simple ROS Noetic example demonstrating how to create and use a **custom message** in Python.  
This project defines a `colour.msg` with RGBA and name fields, builds it with `catkin_make`,  
and includes a publisher node to broadcast colour data and a subscriber node to receive and print it.

---

## 📂 Project Structure
work_mess/
├── src/
│ ├── custom/
│ │ ├── msg/
│ │ │ └── colour.msg
│ │ ├── scripts/
│ │ │ ├── colour_publisher.py
│ │ │ └── colour_subscriber.py
│ │ ├── CMakeLists.txt
│ │ └── package.xml
│ └── pack/ ...


---

## 🛠 Prerequisites
- ROS Noetic (on Ubuntu 20.04)
- Python 3
- Catkin workspace set up

---

## 📦 Build Instructions
1. Clone the repository into your Catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Sanketmail3716/ros-colour-pub-sub.git

---

## 🛠 Prerequisites
- ROS Noetic (on Ubuntu 20.04)
- Python 3
- Catkin workspace set up

---

## 📦 Build Instructions
1. Clone the repository into your Catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Sanketmail3716/ros-colour-pub-sub.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
