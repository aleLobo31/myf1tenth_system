# myf1tenth_system

A collaborative project to develop and simulate autonomous racing algorithms for the [F1TENTH platform](https://f1tenth.org/).

##Collaborators:
- **Alejando Lobo**: [GitHub Profile](https://github.com/aleLobo31)
- **Jaime García Arrojo**: [GitHub Profile](https://github.com/JaimeG-ELC)

##Key Features

- ROS2-based development environment.
- Dockerized setup for easy deployment and reproducibility.
- Tools for building, running, and managing the system.
- Scripts for hardware setup and environment preparation.

##Repository Structure (so far)
.
├── Dockerfile               # Docker setup for ROS2 and project environment
├── LICENSE                  # License for the project
├── README.md                # Project documentation
├── scripts/                 # Helper scripts for setup and management
  └── build_image.sh       # Build Docker image
  └── launch_container.sh  # Launch Docker container
  └── install_docker.sh    # Install Docker and dependencies
  └── set_devices.sh       # Configure hardware devices
  └── create_swapfile.sh   # Create swap file for limited memory systems
├── src/                     # ROS2 packages and source code
  └── f1tenth_stack
  └── manual_control

##Getting Started
- Using the scrips install Docker, ros2

###Docker
- Docker is used to containerized the application, this prevents OS and dependecies issues and while mantaining good performance and efficient resource usage compared to hypervirtualization.
- The Dockerfile is in charge of creating the workspace of the container and loading all the dependencies.
- First an image must be built to run the container, execute 'image_container.sh'
- Then lauch the container: 'launch_container_sh'
