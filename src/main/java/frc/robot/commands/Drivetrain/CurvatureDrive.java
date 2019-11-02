/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;

public class CurvatureDrive extends Command {
  public CurvatureDrive() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Constants.applyDeadband(Robot.m_oi.getDriveController().getY(Hand.kLeft), 0.12) >= 0.12 ) {
    Robot.m_drivetrain.curvatureDrive(Robot.m_oi.getDriveController().getY(Hand.kLeft),
                                       -Robot.m_oi.getDriveController().getX(Hand.kRight), false);
    } else {
      Robot.m_drivetrain.curvatureDrive(Robot.m_oi.getDriveController().getY(Hand.kLeft),
                                       -Robot.m_oi.getDriveController().getX(Hand.kRight), true);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
