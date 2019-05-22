/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.Intakes.DefaultCargoIntakeCommand;

/**
 * Add your docs here.
 */
public class HatchIntake extends Subsystem 
{
  private static HatchIntake hatchIntake = new HatchIntake();

  private DoubleSolenoid m_intakePiston;
  private VictorSPX m_vacuumMotor;

  private boolean m_isVacuumOn = false;
  private boolean m_isIntakePistonExtended = false;

  private HatchIntake()
  {
    m_intakePiston = new DoubleSolenoid(6, 1);
    m_vacuumMotor  = new VictorSPX(14);
  }

  /**
   * function that toggles the position of the piston 
   * on the intake
   */
  public void togglePiston()
  {
    if(m_isIntakePistonExtended)
    {
      retractPiston();  
      m_isIntakePistonExtended = false;
    }else if(!m_isIntakePistonExtended)
    {
      extendPiston();
      m_isIntakePistonExtended = true;
    }
  }

  /**
   * function that toggles the vacuum motor on
   * the intake
   */
  public void toggleVacuum()
  {
    if(m_isVacuumOn)
    {
      disableVacuum();
      m_isVacuumOn = false;
    }else if(!m_isVacuumOn)
    {
      enableVacuum(); 
      m_isVacuumOn = true;
    }
  }

  /**
   * function that extends the piston on the intake
   */
  public void extendPiston()
  {
    m_intakePiston.set(Value.kForward);
  }

  /**
   * function that retracts the piston on the intake
   */
  public void retractPiston()
  {
    m_intakePiston.set(Value.kReverse);
  }

  /**
   * function that turns on the motor for 
   * the vacuum
   */
  public void enableVacuum()
  {
    m_vacuumMotor.set(ControlMode.PercentOutput, 1);
  }

  /**
   * function that turns off the motor
   * for the vacuum
   */
  public void disableVacuum()
  {
    m_vacuumMotor.set(ControlMode.PercentOutput, 0);
  }

  /**
   * static function for HatchIntake singleton
   * @return - static instance of the HatchIntake class
   */
  public static HatchIntake getInstance()
  {
    return hatchIntake;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //setDefaultCommand(new DefaultHatch);
  }
}
