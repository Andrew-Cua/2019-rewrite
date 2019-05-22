/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.StateControl.ArmSetpoints;
import frc.robot.StateControl.WristSetpoints;
import frc.robot.commands.Arm.SetCargoState;
import frc.robot.commands.Drivetrain.MoveDistanceCommand;
import frc.robot.commands.Intakes.AutoScoreCargo;
import frc.robot.commands.Wrist.SetCargoState_Wrist;
import frc.robot.commands.Wrist.ToggleUserControl;

public class AutoGoal extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoGoal() {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
    requires(Robot.m_arm);
    requires(Robot.m_drivetrain);
    requires(Robot.m_wrist);

    addParallel(new MoveDistanceCommand(-21.3));
    //addSequential(new SetCargoState(ArmSetpoints.kCargoShip));
    //addSequential(new ToggleUserControl());
    //addSequential(new AutoScoreCargo());

    //addSequential(new SetCargoState(ArmSetpoints.kNeutral));
    //addParallel(new ToggleUserControl());
  }
}
