/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.StateControl.ArmSetpoints;
import frc.robot.commands.Arm.SetCargoState;
import frc.robot.commands.Arm.SetHatchState;
import frc.robot.commands.Climber.ClimberTestCommand;
import frc.robot.commands.Drivetrain.SeekBallCommand;
import frc.robot.commands.Drivetrain.SeekTapeCommand;
import frc.robot.commands.Drivetrain.ShiftGearCommand;
import frc.robot.commands.Intakes.ActuateIntakeCommand;
import frc.robot.commands.Intakes.CollectCargoCommand;
import frc.robot.commands.Intakes.FullShootCommand;
import frc.robot.commands.Intakes.ShootCargoCommand;
import frc.robot.commands.Intakes.ToggleVacuumCommand;
import frc.robot.commands.Wrist.ToggleUserControl;
import frc.robot.commands.CommandGroups.AutoGoal;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private Joystick intakeStick;
  private XboxController driveController;

  private JoystickButton buttonOne;
  private JoystickButton buttonTwo;
  private JoystickButton buttonThree; 
  private JoystickButton buttonFour;
  private JoystickButton buttonFive; 
  private JoystickButton buttonSix;
  private JoystickButton buttonSeven;
  private JoystickButton buttonEight;
  private JoystickButton buttonNine;
  private JoystickButton buttonTen;
  private JoystickButton buttonEleven;
  private JoystickButton buttonFourteen;
  private JoystickButton buttonThirteen;
  private JoystickButton buttonFifteen;
  private JoystickButton buttonSixteen;


  private JoystickButton x_buttonTwo;
  private JoystickButton x_buttonThree;
  private JoystickButton x_buttonFour;
  private JoystickButton x_buttonOne;
  public OI()
  {
    intakeStick = new Joystick(0);
    driveController = new XboxController(1);
//secret code

    buttonOne = new JoystickButton(intakeStick, 1);
    buttonTwo = new JoystickButton(intakeStick, 2);
    buttonThree = new JoystickButton(intakeStick, 3);
    buttonFour = new JoystickButton(intakeStick, 4);
    buttonFive = new JoystickButton(intakeStick, 5);
    buttonSix  = new JoystickButton(intakeStick, 6);
    buttonSeven = new JoystickButton(intakeStick, 7);
    buttonEight = new JoystickButton(intakeStick, 8);
    buttonNine = new JoystickButton(intakeStick, 9);
    buttonTen = new JoystickButton(intakeStick, 10);
    buttonEleven = new JoystickButton(intakeStick, 11);
    buttonThirteen = new JoystickButton(intakeStick, 13);
    buttonFourteen = new JoystickButton(intakeStick, 14);
    buttonFifteen = new JoystickButton(intakeStick, 15);

    x_buttonTwo = new JoystickButton(driveController, 2);
    x_buttonThree = new JoystickButton(driveController, 3);
    x_buttonFour = new JoystickButton(driveController, 4);
    x_buttonOne = new JoystickButton(driveController, 1);

    buttonOne.whileHeld(new ShootCargoCommand());
    buttonTwo.whenPressed(new SetCargoState(ArmSetpoints.kBallGetter));
    buttonThree.whileHeld(new CollectCargoCommand());   
    buttonFour.whenPressed(new SetCargoState(ArmSetpoints.kCargoShip));
    buttonFive.whenPressed(new SetCargoState(ArmSetpoints.kHighGoal));
    buttonSix.whenPressed(new SetCargoState(ArmSetpoints.kMidGoal));
    buttonSeven.whenActive(new SetCargoState(ArmSetpoints.kLowGoal));
    //buttonEight.whenPressed(new ToggleUserControl());
    buttonNine.whenPressed(new SetCargoState(ArmSetpoints.kNeutral));
    //buttonTen.whenPressed(new SetCargoState(ArmSetpoints.kHighGoal));
    //buttonEleven.whenPressed(new SetCargoState(ArmSetpoints.kNeutral));
    //buttonThirteen.whenPressed(new SetCargoState(ArmSetpoints.kLowGoal));
    buttonFourteen.whileHeld(new FullShootCommand());
    //buttonFifteen.whenPressed(new ToggleVacuumCommand());

    x_buttonTwo.whenPressed(new SeekBallCommand());
    x_buttonThree.whileHeld(new SeekTapeCommand());
    x_buttonFour.whenPressed(new AutoGoal());
    x_buttonOne.whenPressed(new ActuateIntakeCommand());
  }


  /**
   * accessor for the joystick for the arm and wrist
   * @return - the  intakestick object
   */
  public Joystick getIntakeStick()
  {
    return intakeStick;
  }

  /**
   * accessir for the xbox controller for the drivetrain
   * @return - the driveController object
   */
  public XboxController getDriveController()
  {
    return driveController;
  }

}
