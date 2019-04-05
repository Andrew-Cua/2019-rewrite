/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.StateControl.WristSetpoints;
import frc.robot.subsystems.Wrist.TargetPos;

/**
 * Add your docs here.
 */
public class SetHatchState_Wrist extends InstantCommand {
  /**
   * Add your docs here.
   */
  private WristSetpoints m_setpoint;
  public SetHatchState_Wrist(WristSetpoints setpoint) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    m_setpoint = setpoint;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.m_wrist.setTarget(TargetPos.kHatch);
    Robot.m_wrist.setState(m_setpoint);
  }

}
