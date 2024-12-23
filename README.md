# myf1tenth_system

A collaborative project to develop and simulate autonomous racing algorithms for the [F1TENTH platform](https://f1tenth.org/).

## Collaborators:
- **Alejando Lobo**: [GitHub Profile](https://github.com/aleLobo31)
- **Jaime García Arrojo**: [GitHub Profile](https://github.com/JaimeG-ELC)

## Repository Structure (so far)

| Dockerfile
├── LICENSE
├── README.md
├── scripts/
  └── build_image.sh
  └── launch_container.sh
  └── install_docker.sh
  └── set_devices.sh
  └── create_swapfile.sh
├── src/
  └── f1tenth_stack
  └── manual_control

  
## Key Features

- ROS2-based development environment.
- Dockerized setup for easy deployment and reproducibility.
- Tools for building, running, and managing the system.
- Scripts for hardware setup and environment preparation.


### ROS2
- Communication protocol that allows for easydata transfering and processing
- Ease of implementation and scalability
- Standarization system development

### Docker
- Docker is used to containerized the application, this prevents OS and dependecies issues and while mantaining good performance and efficient resource usage compared to hypervirtualization.
- The Dockerfile is in charge of creating the workspace of the container and loading all the dependencies.
- First an image must be built to run the container
- Then the container must be launch to start it

## Getting Started
1.- Using the scrips install Docker, ros2:
`./install_docker_ros2.sh`

2.- Build the image of the container
`./build_container.sh`

3.- Launch the container
`./launch_container.sh`

Everytime the container is going to be launched the image does not have to be rebuilt
If a file (used in the container) is modified, for the changes to be applied, the image must be rebuilt.



