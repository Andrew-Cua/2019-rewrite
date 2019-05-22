/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.StateControl.ArmSetpoints;
import frc.robot.StateControl.WristSetpoints;
import frc.robot.subsystems.Arm.TargetPos;

/**
 * Add your docs here.
 */
public class SetHatchState extends InstantCommand {
  
  ArmSetpoints m_setpoint;
  public SetHatchState(ArmSetpoints setpoint) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_arm);
    m_setpoint = setpoint;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_arm.setTarget(frc.robot.subsystems.Arm.TargetPos.kHatch);
    Robot.m_arm.setState(m_setpoint);
    Robot.m_wrist.setTarget(frc.robot.subsystems.Wrist.TargetPos.kHatch);
    setWristState(m_setpoint);
  }



  private void setWristState(ArmSetpoints setpoint)
  {
    switch(setpoint)
    {
      case kCargoShip:
        Robot.m_wrist.setState(WristSetpoints.kCargoShip);
        break;
      case kHighGoal:
        Robot.m_wrist.setState(WristSetpoints.kHigh);
        break;
      case kLowGoal:
        Robot.m_wrist.setState(WristSetpoints.kLow);
      case kMidGoal:
        Robot.m_wrist.setState(WristSetpoints.kMid);
        break;
      case kNeutral:
        Robot.m_wrist.setState(WristSetpoints.kNeutral);
        break;
      case kBallGetter:
        Robot.m_wrist.setState(WristSetpoints.kBallGetter);
    }
  }

  

}
