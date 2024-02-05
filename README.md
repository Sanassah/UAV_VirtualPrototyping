# Hexacopter Simulation using MATLAB

This repository contains a comprehensive hexacopter simulation implemented in MATLAB. The simulation includes a custom plant model, control allocation algorithm, and various control laws to accurately mimic the behavior of a hexacopter in a virtual environment.

![image](https://github.com/Sanassah/UAV_VirtualPrototyping/assets/89364990/ad8b4247-c6fb-47f3-bb75-c9027eecb259)

## Features


- Custom Hexacopter Plant: The simulation includes a detailed and customizable hexacopter plant model that takes into account the dynamics and characteristics of a real hexacopter. You can tweak the parameters to match your specific hexacopter configuration.

- Control Allocation: The project incorporates a control allocation algorithm that efficiently distributes control commands to the individual actuators of the hexacopter. This allows for precise control and maneuverability.

- Control Laws: Various control laws have been implemented to govern the hexacopter's behavior. These laws include PID control, trajectory tracking, altitude control, and more. You can experiment with different control laws or even create your own to achieve desired flight characteristics.
  
- Reliability Model: Allows a different distribution of power to each motor depending on their health.

![Drone_Simulation](https://github.com/Sanassah/UAV_VirtualPrototyping/assets/89364990/01042729-8d4d-4604-b1e1-b74a2f55a672)


![IMG_0 (1)](https://github.com/Sanassah/UAV_VirtualPrototyping/assets/89364990/d6828937-239a-4580-ba77-0a7cb60ae23a)
Flight test of physical twin with custom made control laws integrated to the pixhawk 4 flight controller through the px4 support package of matlab.

## Getting Started

### Prerequisites

1. MATLAB 2020a (Available in all labs of Concordia University)
2. Simscape Add-On (Available in Concordia University, Dr. Jonathan Liscouet Lab)
3. Simscape Multibody Add-On (Available in Concordia University, Dr. Jonathan Liscouet Lab)
4. Simscape Electrical Add-On (Available in Concordia University EV building 8th floor labs)


### Installation

1. Download the main repository

2. Open MATLAB 2020a or newer version and navigate to the cloned repository folder.
   
3. Open the project simulation file, `UAV_VirtualPrototyping.prj`, in MATLAB.

4. Enter a flight plan in the popup app and run the project



For any questions or inquiries, feel free to reach out to anas.senouci@gmail.com

Happy flying!
