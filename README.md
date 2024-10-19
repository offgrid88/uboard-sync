# uBoardSync

**uBoardSync** is a ROS 2 Humble node designed to manage micro-ROS agents for multiple boards seamlessly. It automatically detects new boards, creates and runs a ROS agent for each, monitors board statuses to stop agents upon disconnection, and manages udev rules for device identification. This tool simplifies the integration of multiple micro-ROS devices into your ROS 2 ecosystem.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Udev Rule Manager](#udev-rule-manager)
- [Topics](#topics)
- [Example](#example)
- [Contributing](#contributing)
- [License](#license)

## Features

- **Automatic Board Detection**: Detects when new micro-ROS boards are connected.
- **Dynamic Agent Management**: Creates and runs a micro-ROS agent for each detected board.
- **Board Monitoring**: Monitors board status and stops the agent if a board disconnects.
- **Udev Rule Management**: Automatically creates udev rules and symlinks based on provided PID and VID.
- **Configurable Boards List**: Uses a configuration file to manage board settings.
- **ROS 2 Topics**: Publishes the status of connected boards on specific ROS 2 topics.

## Prerequisites

- **Operating System**: Ubuntu 22.04 or compatible Linux distribution.
- **ROS 2 Distribution**: ROS 2 Humble Hawksbill.
- **Python**: Python 3.8 or higher.
- **micro-ROS Agent**: Installed and accessible in your environment.

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/offgrid88/uBoardSync.git
cd uBoardSync
```

### 2. Build the Package

Assuming you have a ROS 2 workspace set up:

```bash
cd ~/your_ros2_workspace/src
ln -s /path/to/uBoardSync .
cd ~/your_ros2_workspace
colcon build --packages-select uboardsync
```

### 3. Source the Workspace

```bash
source ~/your_ros2_workspace/install/setup.bash
```

### 4. Install Dependencies

Install any Python dependencies using `pip`:

```bash
pip install -r uboardsync/requirements.txt
```

*Note: Ensure that `pip` installs packages for the Python version used by ROS 2.*

## Usage

### 1. Configure Boards

Edit the `boards_config.yaml` file to include your boards' information. See the [Configuration](#configuration) section for details.

### 2. Run uBoardSync

```bash
ros2 run uboardsync uboardsync_node
```

This command starts the uBoardSync node, which begins monitoring for connected boards as per your configuration.

## Configuration

The `boards_config.yaml` file holds the configuration for the boards you wish to manage. It should be located in the `config` directory of the package.

### Sample `boards_config.yaml`

```yaml
boards:
  - name: board_1
    pid: '0x2341'
    vid: '0x0043'
    serial_number: 'ABC12345'
    agent_port: '/dev/uboard_board_1'
    namespace: 'board_1_ns'
  - name: board_2
    pid: '0x2341'
    vid: '0x0043'
    serial_number: 'DEF67890'
    agent_port: '/dev/uboard_board_2'
    namespace: 'board_2_ns'
```

### Configuration Parameters

- **name**: A unique identifier for the board.
- **pid**: USB Product ID of the board (hexadecimal string, e.g., `'0x2341'`).
- **vid**: USB Vendor ID of the board (hexadecimal string, e.g., `'0x0043'`).
- **serial_number**: Serial number of the board (optional but recommended for precise identification).
- **agent_port**: The symlinked device path that udev will create (e.g., `'/dev/uboard_board_1'`).
- **namespace**: The ROS 2 namespace to be used by the agent.

## Udev Rule Manager

The udev rule manager automates the creation of udev rules to assign consistent device names (symlinks) based on PID and VID, and optionally the serial number.

### Setting Up Udev Rules

uBoardSync will attempt to set up udev rules automatically based on your configuration. Ensure you have the necessary permissions:

```bash
sudo usermod -a -G dialout $USER
```

After modifying groups, you may need to log out and log back in.

### Manual Udev Rule Creation (Optional)

If automatic setup fails, you can manually create udev rules:

1. Create a udev rules file:

   ```bash
   sudo nano /etc/udev/rules.d/99-uboard.rules
   ```

2. Add rules for your devices:

   ```udev
   SUBSYSTEM=="tty", ATTR{idProduct}=="2341", ATTR{idVendor}=="0043", ATTR{serial}=="ABC12345", SYMLINK+="uboard_board_1"
   SUBSYSTEM=="tty", ATTR{idProduct}=="2341", ATTR{idVendor}=="0043", ATTR{serial}=="DEF67890", SYMLINK+="uboard_board_2"
   ```

3. Reload udev rules:

   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

## Topics

uBoardSync publishes the status of connected boards on ROS 2 topics:

- **`/uboardsync/connected_boards`** (`std_msgs/String`): Lists the names of currently connected boards.
- **`/uboardsync/board_status/<board_name>`** (`std_msgs/Bool`): Indicates whether a specific board is connected (`True`) or disconnected (`False`).

## Example

### Running the Node

After configuring your boards, run the node:

```bash
ros2 run uboardsync uboardsync_node
```

### Monitoring Topics

You can check the connected boards:

```bash
ros2 topic echo /uboardsync/connected_boards
```

Monitor a specific board's status:

```bash
ros2 topic echo /uboardsync/board_status/board_1
```

### Interacting with Agents

Agents for each board are namespaced as per your configuration. You can interact with them using standard ROS 2 tools.

## Contributing

Contributions are welcome! If you encounter issues or have suggestions, please open an issue or submit a pull request.

### Steps to Contribute

1. Fork the repository.
2. Create a new branch:

   ```bash
   git checkout -b feature/your_feature
   ```

3. Make your changes.
4. Commit and push:

   ```bash
   git commit -m "Description of your changes"
   git push origin feature/your_feature
   ```

5. Open a pull request on GitHub.

## License

This project is licensed under the [MIT License](LICENSE).
