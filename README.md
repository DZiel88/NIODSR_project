<<<<<<< HEAD
# NIODSR_project

1. Będąc w folderze (aby pobrać paczkę zawierającą symulator) ./src wpisz: git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
2. Skonfiguruj środowisko: source install/setup.bash
3. Wybierz model robota:
export TURTLEBOT3_MODEL=waffle lub export TURTLEBOT3_MODEL=burger
4. Uruchom paczkę symulatora z wybranym środowiskiem: 
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py 
5. Uruchom nowy terminal i skonfiguruj go: source install/setup.bash
6. Uruchom węzeł odpowiedzialny za sterowanie robotem: ros2 run turtlebot_control turtlebot3_ctl
7. Uruchom nowy terminal i skonfiguruj go: source install/setup.bash
8. Uruchom węzeł odpowiedzialny za odczyt prędkości robota: ros2 run turtlebot_control turtlebot3_timer 

