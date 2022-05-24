import roboticstoolbox as rtb
from roboticstoolbox.models.URDF.UR5 import UR5
robot = UR5()
print(robot.jacob0([0,0,0,0,0,0]))
