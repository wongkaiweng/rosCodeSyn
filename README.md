# rosCodeSyn - A ROS Code Conversion Program

rosCodeSyn is a standalone Python library that allows user to convert a Python script for one robot to a script for another robot.

rosCodeSyn includes [kdl_retargeter](https://github.com/wongkaiweng/rosCodeSyn/wiki/kdl_retargeting) by Tarik Tosun et. al.

## Installation Requirements
To run rosCodeSyn, you need the following Python dependencies: 

* [urdf_parser_py](https://github.com/ros/urdf_parser_py)
* [pykdl_utils](http://wiki.ros.org/pykdl_utils) - Source on Git
* PyKDL `sudo apt-get install ros-indigo-python-orocos-kdl`
* redbaron, pyyaml, matplotlib, pandas, numpy, scipy `pip install redbaron pyyaml matplotlib pandas numpy scipy`

(Optional) ROS `sudo apt-get install ros-indigo-desktop`

## Running rosCodeSyn
1. Run `python src/code_synthesis.py`
2. Choose an example from the list. Enter the example number in the command prompt.
3. The program may prompt for other selections such as topic selections or parameter selections.
4. When finished, you can find the generated Python script in a corresponding example folder.

You can find more examples in the `examples` folder.

## Authors

Catherine Wong and Hila Peleg

See also the list of [contributors](https://github.com/wongkaiweng/rosCodeSyn/contributors) who participated in this project.

## License

This project is licensed under the GNUv3 License - see the [LICENSE](LICENSE) file for details
