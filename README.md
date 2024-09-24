[![roboticslab-uc3m logo](fig/roboticslab-banner-350px.png)](https://github.com/roboticslab-uc3m)

# SOFA - Soft Robotics

This repository contains the necessary files for running inverse and direct kinematics with several soft robotic platforms developed in the Soft Robotics research group of the RoboticsLab. These examples are prepared to be run on SOFA 23.12.01, using a different version will require you to change dependencies for some plugins.

In order to run the examples from a different folder outside the SOFA examples for each plugin you need to add the following lines to the bashrc, changing **X** for your username.

```bash
alias runSofa='/home/X/SOFA/v23.12.01/bin/runSofa'
export SOFA_ROOT=/home/X/SOFA/v23.12.01
export PYTHONPATH="$SOFA_ROOT/plugins/SofaPython3/lib/python3/site-packages:$SOFA_ROOT/plugins/SoftRobots/lib/python3/site-packages:$SOFA_ROOT/plugins/STLIB/lib/python3/site-packages:$SOFA_ROOT/plugins/Cosserat/lib/python2.7/site-packages:$PYTHONPATH"
```

### Acknowledgements

The [SoftRobots](https://github.com/SofaDefrost/SoftRobots) and [Cosserat](https://github.com/SofaDefrost/Cosserat?tab=readme-ov-file) plugins used in these examples have been developed by [SofaDefrost](https://github.com/SofaDefrost) team. Further documentation and examples can be found on their respective repositories.