# Demo ctl
#
# source: /home/karpov/ros/src/kvorum2/usr/examples/03-sensors/config/field.map
# dimX = 100 dimY = 100 step_x = 2.00 step_y = 3.12
#

# Robots positions
_RobotNum_ = 1
_RobotX_ = [48]
_RobotY_ = [18]
_RobotA_ = [90]


Env.SetRectangleField(34, 28, 36, 31, gdic.LEVEL_GROUND, 1)

Env.SetRectangleField(36, 28, 38, 31, gdic.LEVEL_GROUND, 1)

Env.SetRectangleField(42, 28, 44, 31, gdic.LEVEL_GROUND, 1)

Env.SetRectangleField(44, 28, 46, 31, gdic.LEVEL_GROUND, 1)

Env.SetRectangleField(50, 25, 52, 28, gdic.LEVEL_LIGHT, 1)

Env.SetRectangleField(52, 25, 54, 28, gdic.LEVEL_LIGHT, 1)

Env.SetRectangleField(56, 25, 58, 28, gdic.LEVEL_COLOR, 10)
Env.Text(56, 28, "10")

Env.SetRectangleField(34, 18, 36, 21, gdic.LEVEL_IR, 1)
Env.Text(34, 21, "1")

Env.SetRectangleField(60, 18, 62, 21, gdic.LEVEL_IR, 3)
Env.Text(60, 21, "3")

Env.SetRectangleField(48, 12, 50, 15, gdic.LEVEL_IR, 2)
Env.Text(48, 15, "2")

