# Micromouse
An autonomous maze-solving robot using ESP32 microcontroller and a custom PCB.

A Micromouse is a small autonomous robot that must navigate and solve an unknown maze as quickly as possible. While traversing through a maze, our prototype micromouse runs on a "left-turn" algorithm, that turns left at every possible left turn existing, which ensures that the micromouse will find a solution.

**Components used:**

1) ESP32 microcontroller
2) VL53L0X Lidar Time-of-flight sensors
3) MPU6500 Gyroscope
4) 3V LiPo Battery
5) MT3608 Boost Converter
6) N20 Motors
7) DRV8833 Dual channel motor driver
8) N20 Wheels

The robot provides a foundation for implementing advanced path-planning algorithms in future iterations, such as Flood Fill, Dijkstra’s Algorithm, and A* which can be incorporated to improve maze exploration efficiency and optimize route selection.
