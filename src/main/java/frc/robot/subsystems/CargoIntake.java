/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.Intakes.DefaultCargoIntakeCommand;

/**
 * Add your docs here.
 */
public class CargoIntake extends Subsystem 
{
  private static CargoIntake cargoIntake = new CargoIntake();

  private TalonSRX m_intakeMotor;
  public CargoIntake()
  {
    m_intakeMotor = new TalonSRX(12);
    m_intakeMotor.setInverted(InvertType.InvertMotorOutput);
  }

  /**
   * function that runs the motor to 
   * to collect cargo from the floor
   */
  public void collectCargo()
  {
    m_intakeMotor.set(ControlMode.PercentOutput, 0.25);
  }

  /**
   * function that runs the motor to
   * shoot the cargo into the goal
   */
  public void shootCargo()
  {

    m_intakeMotor.set(ControlMode.PercentOutput, -0.25);
    
  }

  public void fullShoot()
  {
    m_intakeMotor.set(ControlMode.PercentOutput, -1);
  }


  /**
   * useless dont use
   */
  public void timedShoot()
  {
    double startTime = Timer.getFPGATimestamp();
    double currentTime = Timer.getFPGATimestamp();
    while((currentTime - startTime) < 0.5)
    {
      shootCargo();
    } 

    stopMotor();
  }

  /**
   * function that stops the motor
   */
  public void stopMotor()
  {
    m_intakeMotor.set(ControlMode.PercentOutput, 0);
  }


  /**
   * static function for the CargoIntake singleton
   * @return - returns static instance of the CargoIntake class
   */
  public static CargoIntake getInstance()
  {
    return cargoIntake;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefaultCargoIntakeCommand());
  }
}
