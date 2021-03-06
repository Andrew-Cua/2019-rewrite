/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class EvoShifters extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static EvoShifters m_shifters = new EvoShifters();

  private Solenoid m_shifterOut;
  private Solenoid m_shifterIn;
  private boolean isLowGear = false;
  private EvoShifters()
  {
    m_shifterOut = new Solenoid(6);
    m_shifterIn = new Solenoid(3);
  }



  /**
   * function that toggles the gear position of the robot
   */
  public void toggleShift()
  {
    if(isLowGear)
    {
      highGear();
      isLowGear = false;
      SmartDashboard.putString("gear level", "High Gear");
    }else if(!isLowGear)
    {
      lowGear();
      isLowGear = true;
      SmartDashboard.putString("gear level", "Low Gear");
    }
  }

  /**
   * function that shifts the gearbox into highgear 
   * robot is in highgear by default
   */
  private void highGear()
  {
    m_shifterOut.set(false);
    m_shifterIn.set(true);
  }

  /**
   * function that shifts the gearbox into lowgear
   */
  private void lowGear()
  {
    m_shifterOut.set(true);
    m_shifterIn.set(false);
  }


  /**
   * static function for shifter singleton
   * @return - returns static instance of the Evoshifter class
   */
  public static EvoShifters getInstance()
  {
    return m_shifters;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
