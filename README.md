# motion_control
ROS package for autonomous and manual driving

# Install package

1. Clone the repository (recommended location is ~).

  ```bash
  cd ~
  ```

  ```bash
  git clone https://github.com/robocol-rem-u/motion_control.git
  ```

2. Move to the root of the workspace.

  ```bash
  cd ~/motion_control
  ```

3. Build the workspace

  ```bash
  source devel/setup.bash
  ```

4. In a new tab (Ctrl+Shift+t) run _roscore_

  ```bash
  roscore
  ```

5. Run the desired node in your previous tab.

  ### Planning node
  ```bash
  rosrun motion_control_pkg planeacion_a.py
  ```
