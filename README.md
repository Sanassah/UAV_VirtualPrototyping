# Hexacopter Simulation using MATLAB

This repository contains a comprehensive hexacopter simulation implemented in MATLAB. The simulation includes a custom plant model, control allocation algorithm, and various control laws to accurately mimic the behavior of a hexacopter in a virtual environment.

## Features

- Custom Hexacopter Plant: The simulation includes a detailed and customizable hexacopter plant model that takes into account the dynamics and characteristics of a real hexacopter. You can tweak the parameters to match your specific hexacopter configuration.

- Control Allocation: The project incorporates a control allocation algorithm that efficiently distributes control commands to the individual actuators of the hexacopter. This allows for precise control and maneuverability.

- Control Laws: Various control laws have been implemented to govern the hexacopter's behavior. These laws include PID control, trajectory tracking, altitude control, and more. You can experiment with different control laws or even create your own to achieve desired flight characteristics.
- 
- Reliability Model: Allows a different distribution of power to each motor depending on their health.

## Getting Started

### Prerequisites

1. MATLAB 2020a
2. Simscape Add-On
3. Simscape Multibody Add-On
4. Simscape Electrical Add-On
5. Simscape Driveline Add-On


### Installation

1. Download the main repository

2. Open MATLAB and navigate to the cloned repository folder.

3. Open the project simulation file, `UAV_VirtualPrototyping.prj`, in MATLAB.

4. Enter a flight plan in the popup app and run the project


## Contributing

Contributions to this hexacopter simulation project are welcome! If you find any bugs or have ideas for improvements, please submit an issue or pull request. Make sure to follow the established code style and provide clear documentation for any changes you propose.


For any questions or inquiries, feel free to reach out to anas.senouci@gmail.com

Happy flying!
