# Line-Directed Robot
Embedded code for a 2-wheeled line-following robot (written in C using Atmel Studio).

The robot used is the [Sparkfun Redbot](https://www.sparkfun.com/products/12649).
It uses three IR sensors in the front to follow a line of black tape, and has two motorized wheels for directional control. The entire source code for the project is contained within `redbot_new`. The `main.c` code can be modified to change the speed, sensitivity, and other factors of the robot. This project requires installation of `avrdude`. Can be opened in Atmel Studio through `redbot_new.atsln`.

<p align="center">
  <img src="https://cdn.sparkfun.com/assets/parts/1/0/9/2/0/13582-05.jpg">
</p>
