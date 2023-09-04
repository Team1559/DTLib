package org.victorrobotics.frc.dtlib.log;

public class DTLogRobotNode extends DTLogTreeNode {
  private final Object   robot;
  private final Class<?> clazz;

  public DTLogRobotNode(Object robot) {
    super(robot.getClass()
               .getSimpleName());
    this.robot = robot;
    this.clazz = robot.getClass();
  }

  @Override
  protected Class<?> getType() {
    return clazz;
  }

  @Override
  protected Object getValue(Object parent) {
    return robot;
  }
}
