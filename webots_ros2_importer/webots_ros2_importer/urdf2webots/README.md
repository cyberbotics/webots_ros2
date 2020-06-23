# urdf2webots

[![Build Status](https://travis-ci.com/cyberbotics/urdf2webots.svg?branch=master)](https://travis-ci.com/cyberbotics/urdf2webots)

This tool converts URDF files into Webots PROTO files.

## Install

### From pip

```
pip install urdf2webots
```

On macOS, export the pip binary path to the PATH: `export PATH="/Users/$USER/Library/Python/3.7/bin:$PATH"`

### From Sources

```
git clone https://github.com/cyberbotics/urdf2webots.git
cd urdf2webots
pip install -r requirements.txt
```

## Usage

### From pip

```
python -m urdf2webots.importer --input=someRobot.urdf [--output=outputFile] [--box-collision] [--normal] [--disable-mesh-optimization]
```

### From Sources

```
python demo.py --input=someRobot.urdf [--output=outputFile] [--box-collision] [--normal] [--disable-mesh-optimization]
```

### In your Python Code

```
from urdf2webots.importer import convert2urdf
convert2urdf('MY_PATH/MY_URDF.urd')
```

### Arguments

Outputs: someRobot_textures (folder), someRobot.proto.  
Use in Webots: put the outputs in the protos folder of your Webots project.

## Notes
This tool have been tested using Webots R2020a on Ubuntu16.04 and Windows.  
You can find the sources of these URDF files here:  
  - universal robot: https://github.com/ros-industrial/universal_robot/tree/kinetic-devel/ur_description  
  - pr2 robot: https://github.com/PR2/pr2_common/tree/kinetic-devel/pr2_description  
  - motoman robot: https://github.com/ros-industrial/motoman/tree/kinetic-devel/motoman_sia20d_support
  - kinova robot: https://github.com/Kinovarobotics/kinova-ros/tree/kinetic/kinova_description
  - gait2392 human skeleton: https://github.com/cyberbotics/urdf2webots/tree/master/tests/sources/gait2392_simbody

## Acknowledgement

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png"
       alt="rosin_logo" height="60" >
</a></br>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg"
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287.
