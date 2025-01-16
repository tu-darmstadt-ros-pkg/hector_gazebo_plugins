# Common Gazebo Plugins

## Available Plugins

    - MimicJoints


## Build new Plugins

1. Add plugin and dependencies to CmakeLists.txt

2. Build this package as usual

This will generate the `<pluginName>.so` library under `install/hector_gazebo_plugins/plugins`

## Run

1. Include plugin in robot URDF or world file (see Hector Gazebo Harmonic documentation)

2. Add this repo to your ROS 2 src directory if you have not done so

3. Source install/setup.bash to export plugin(s) to GZ_SIM_SYSTEM_PLUGIN_PATH

4. Run sim
