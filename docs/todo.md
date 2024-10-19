
# TODO List for uBoardSync

**uBoardSync** is a C++ ROS 2 Humble node designed to manage micro-ROS agents for multiple boards seamlessly. This document outlines the tasks required to complete the project, from initial setup to deployment.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Phase 1: Planning and Setup](#phase-1-planning-and-setup)
  - [1.1 Define Project Scope](#11-define-project-scope)
  - [1.2 Set Up Development Environment](#12-set-up-development-environment)
  - [1.3 Create Initial Project Structure](#13-create-initial-project-structure)
- [Phase 2: Core Development](#phase-2-core-development)
  - [2.1 Implement Configuration Loader](#21-implement-configuration-loader)
  - [2.2 Develop Udev Manager](#22-develop-udev-manager)
  - [2.3 Build Board Detector](#23-build-board-detector)
  - [2.4 Create Agent Manager](#24-create-agent-manager)
  - [2.5 Develop Main Node](#25-develop-main-node)
  - [2.6 Implement Namespace Isolation](#26-implement-namespace-isolation)
- [Phase 3: Enhanced Features](#phase-3-enhanced-features)
  - [3.1 Dynamic Configuration Reload](#31-dynamic-configuration-reload)
  - [3.2 Error Handling and Recovery](#32-error-handling-and-recovery)
  - [3.3 Logging and Monitoring](#33-logging-and-monitoring)
  - [3.4 Command-Line Interface Options](#34-command-line-interface-options)
  - [3.5 Resource Optimization](#35-resource-optimization)
- [Phase 4: Testing and Validation](#phase-4-testing-and-validation)
  - [4.1 Unit Testing](#41-unit-testing)
  - [4.2 Integration Testing](#42-integration-testing)
  - [4.3 Continuous Integration Setup](#43-continuous-integration-setup)
- [Phase 5: Documentation](#phase-5-documentation)
  - [5.1 Code Documentation](#51-code-documentation)
  - [5.2 User Guide](#52-user-guide)
  - [5.3 Contribution Guidelines](#53-contribution-guidelines)
- [Phase 6: Deployment and Release](#phase-6-deployment-and-release)
  - [6.1 Dockerization](#61-dockerization)
  - [6.2 Release Management](#62-release-management)
  - [6.3 Publishing Package](#63-publishing-package)
- [Phase 7: Future Enhancements](#phase-7-future-enhancements)
  - [7.1 GUI Development](#71-gui-development)
  - [7.2 Web-Based Dashboard](#72-web-based-dashboard)
  - [7.3 Multi-Platform Support](#73-multi-platform-support)
  - [7.4 Security Enhancements](#74-security-enhancements)

---

## Project Overview

**Objective**: Develop a ROS 2 Humble node that automates the detection of micro-ROS boards, manages individual agents for each board, monitors their status, and handles udev rules for device identification.

---

## Phase 1: Planning and Setup

### 1.1 Define Project Scope

- **Tasks**:
  - [X] Clearly define the objectives and functionalities of uBoardSync.
  - [X] List out all features to be included in the initial release.
  - [X] Identify any dependencies or external libraries required.

### 1.2 Set Up Development Environment

- **Tasks**:
  - [X] Install ROS 2 Humble Hawksbill.
  - [X] Set up a C++ development environment with necessary tools (e.g., GCC, CMake).
  - [X] Install micro-ROS agent and ensure it's accessible.
  - [X] Verify that the development machine has the required permissions for udev and serial communication.

### 1.3 Create Initial Project Structure

- **Tasks**:
  - [X] Initialize a new ROS 2 package named `uBoardSync`.
  - [X] Set up the directory structure as outlined in the project structure.
  - [X] Create essential files: `CMakeLists.txt`, `package.xml`, `.gitignore`, and `LICENSE`.
  - [X] Initialize a Git repository and make the first commit.

---

## Phase 2: Core Development

### 2.1 Implement Configuration Loader

- **Tasks**:
  - [ ] Define the structure of `boards_config.yaml`.
  - [ ] Write `config_loader.hpp` and `config_loader.cpp` to parse the YAML file.
    - [ ] Use a YAML parsing library (e.g., `yaml-cpp`).
    - [ ] Validate the configuration data.
  - [ ] Handle exceptions and invalid configurations.

### 2.2 Develop Udev Manager

- **Tasks**:
  - [ ] Create `udev_manager.hpp` and `udev_manager.cpp`.
  - [ ] Implement functions to generate udev rules based on PID, VID, and serial numbers.
  - [ ] Write logic to create symlinks for devices.
  - [ ] Ensure udev rules are applied and devices are correctly symlinked.
  - [ ] Handle permissions and require sudo privileges where necessary.
  - [ ] Implement backup and restore mechanisms for existing udev rules.

### 2.3 Build Board Detector

- **Tasks**:
  - [ ] Create `board_detector.hpp` and `board_detector.cpp`.
  - [ ] Implement functionality to detect when boards are connected or disconnected.
    - [ ] Use Linux APIs or monitor `/dev` directory changes.
  - [ ] Interface with the udev manager to recognize devices.
  - [ ] Emit signals or callbacks when a board's status changes.

### 2.4 Create Agent Manager

- **Tasks**:
  - [ ] Develop `agent_manager.hpp` and `agent_manager.cpp`.
  - [ ] Implement methods to start and stop micro-ROS agents for each board.
    - [ ] Use `std::system` or `boost::process` to manage agent processes.
  - [ ] Assign appropriate namespaces to each agent.
  - [ ] Monitor agent processes and restart if necessary.
  - [ ] Ensure agents are terminated when boards disconnect.

### 2.5 Develop Main Node

- **Tasks**:
  - [ ] Write `uboard_sync_node.hpp` and `uboard_sync_node.cpp`.
  - [ ] Initialize ROS 2 node and executor.
  - [ ] Integrate the configuration loader, udev manager, board detector, and agent manager.
  - [ ] Implement ROS 2 topics to publish connected boards and their statuses.
    - [ ] `/uboardsync/connected_boards` (`std_msgs::String`)
    - [ ] `/uboardsync/board_status/<board_name>` (`std_msgs::Bool`)
  - [ ] Handle ROS 2 parameters and command-line arguments.

### 2.6 Implement Namespace Isolation

- **Tasks**:
  - [ ] Ensure each agent operates within its own ROS namespace.
  - [ ] Modify agent startup commands to include namespace arguments.
  - [ ] Update documentation to reflect namespace usage.

---

## Phase 3: Enhanced Features

### 3.1 Dynamic Configuration Reload

- **Tasks**:
  - [ ] Implement a file watcher in `config_loader` to detect changes in `boards_config.yaml`.
    - [ ] Use `inotify` on Linux for file system notifications.
  - [ ] Reload configurations without restarting the node.
  - [ ] Update agents based on new configurations.
    - [ ] Start agents for newly added boards.
    - [ ] Stop agents for removed boards.
  - [ ] Ensure thread safety during the reload process.

### 3.2 Error Handling and Recovery

- **Tasks**:
  - [ ] Implement try-catch blocks where exceptions might occur.
  - [ ] Provide meaningful error messages and log them using ROS 2 logging.
  - [ ] Implement retry mechanisms for transient errors (e.g., temporary USB disconnects).
  - [ ] Ensure the node continues running even if some agents fail.

### 3.3 Logging and Monitoring

- **Tasks**:
  - [ ] Use ROS 2 logging macros for consistent logging (`RCLCPP_INFO`, `RCLCPP_ERROR`, etc.).
  - [ ] Log significant events such as:
    - [ ] Board connections/disconnections.
    - [ ] Agent startups and shutdowns.
    - [ ] Errors and warnings.
  - [ ] Optionally integrate with external monitoring tools or ROS 2 diagnostics.

### 3.4 Command-Line Interface Options

- **Tasks**:
  - [ ] Add command-line arguments to control node behavior.
    - [ ] `--config-file` to specify a custom configuration file.
    - [ ] `--log-level` to set the logging verbosity.
    - [ ] `--help` to display available options.
  - [ ] Parse arguments using ROS 2 parameters or `rclcpp::NodeOptions`.

### 3.5 Resource Optimization

- **Tasks**:
  - [ ] Optimize the use of threads and processes.
    - [ ] Use asynchronous I/O operations where applicable.
  - [ ] Profile the application to identify bottlenecks.
    - [ ] Use profiling tools like Valgrind or gprof.
  - [ ] Optimize memory and CPU usage, especially when managing multiple boards.

---

## Phase 4: Testing and Validation

### 4.1 Unit Testing

- **Tasks**:
  - [ ] Write unit tests for individual classes and methods.
    - [ ] Use `ament_cmake_gtest` for C++ unit tests.
  - [ ] Test cases for:
    - [ ] Configuration loading and parsing.
    - [ ] Udev rule generation.
    - [ ] Board detection logic.
    - [ ] Agent management functions.
  - [ ] Mock external dependencies where necessary.

### 4.2 Integration Testing

- **Tasks**:
  - [ ] Write integration tests to simulate real-world scenarios.
    - [ ] Use ROS 2 launch testing framework.
  - [ ] Test the entire workflow:
    - [ ] Board connections and disconnections.
    - [ ] Agent startups and shutdowns.
    - [ ] Topic publishing and subscriptions.
  - [ ] Validate that the system handles multiple boards correctly.

### 4.3 Continuous Integration Setup

- **Tasks**:
  - [ ] Set up a CI pipeline using GitHub Actions.
    - [ ] Configure the workflow to build the package on push and pull requests.
    - [ ] Run unit and integration tests automatically.
    - [ ] Include linters like `ament_cpplint` and `ament_clang_format`.
  - [ ] Ensure the CI environment mimics the target deployment environment.

---

## Phase 5: Documentation

### 5.1 Code Documentation

- **Tasks**:
  - [ ] Comment code thoroughly using Doxygen-style comments.
    - [ ] Document classes, methods, and important code blocks.
  - [ ] Generate API documentation using Doxygen.
  - [ ] Host the documentation on GitHub Pages or a documentation site.

### 5.2 User Guide

- **Tasks**:
  - [ ] Write a comprehensive README.md explaining:
    - [ ] Project overview.
    - [ ] Installation instructions.
    - [ ] Usage examples.
    - [ ] Configuration details.
  - [ ] Include examples and screenshots where helpful.
  - [ ] Provide troubleshooting tips for common issues.

### 5.3 Contribution Guidelines

- **Tasks**:
  - [ ] Create a `CONTRIBUTING.md` file outlining:
    - [ ] Code style guidelines.
    - [ ] Branching and commit message conventions.
    - [ ] Pull request procedures.
    - [ ] Issue reporting guidelines.
  - [ ] Encourage community contributions by being welcoming and providing clear instructions.

---

## Phase 6: Deployment and Release

### 6.1 Dockerization

- **Tasks**:
  - [ ] Create a `Dockerfile` to containerize the application.
    - [ ] Use an official ROS 2 base image.
    - [ ] Install necessary dependencies within the container.
    - [ ] Configure the container to access USB devices (e.g., using `--device` flags).
  - [ ] Test the container thoroughly to ensure it functions as expected.
  - [ ] Provide documentation on how to build and run the Docker container.

### 6.2 Release Management

- **Tasks**:
  - [ ] Tag versions in Git to mark release points.
  - [ ] Update `package.xml` with the correct version numbers.
  - [ ] Create release notes summarizing changes and new features.
  - [ ] Optionally automate release generation using GitHub Releases.

### 6.3 Publishing Package

- **Tasks**:
  - [ ] Prepare the package for publishing to ROS 2 package repositories if desired.
  - [ ] Ensure all dependencies are correctly listed in `package.xml`.
  - [ ] Follow ROS 2 community guidelines for package submission.

---

## Phase 7: Future Enhancements

### 7.1 GUI Development

- **Tasks**:
  - [ ] Develop a graphical user interface to monitor and control uBoardSync.
    - [ ] Choose a GUI framework (e.g., Qt, GTK).
    - [ ] Display connected boards and their statuses.
    - [ ] Provide controls to start/stop agents manually.
  - [ ] Integrate the GUI with the ROS 2 node.

### 7.2 Web-Based Dashboard

- **Tasks**:
  - [ ] Create a web interface for remote monitoring.
    - [ ] Use web technologies like React or Vue.js.
    - [ ] Set up a backend server to interface with the ROS 2 node.
  - [ ] Implement real-time updates using WebSockets.

### 7.3 Multi-Platform Support

- **Tasks**:
  - [ ] Adapt the application to support other operating systems.
    - [ ] For Windows: Handle device detection without udev.
    - [ ] For macOS: Use appropriate APIs for device management.
  - [ ] Abstract OS-specific code to allow for easier maintenance.

### 7.4 Security Enhancements

- **Tasks**:
  - [ ] Implement ROS 2 security features (SROS2) for encrypted communication.
  - [ ] Ensure file permissions are set correctly for udev rules and logs.
  - [ ] Conduct a security audit to identify and fix vulnerabilities.

