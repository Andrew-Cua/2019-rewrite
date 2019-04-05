/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.Climber.ClimberTestCommand;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Climber instance = new Climber();

  private CANSparkMax m_climberMotor; 
  private CANPIDController m_climbPID;
  private CANEncoder m_climbEncoder;

  private DoubleSolenoid m_frontSolenoid;
  private boolean m_frontExtended = false;
  private boolean m_isSecond = false;

  private enum climbLevel
  {
    kSecondLvl(10), kThirdLvl(15);
    private double revLevel;
    private climbLevel(double level)
    {
      revLevel = level;
    }

    public double getLevel()
    {
      return revLevel;
    }
  }
  private Climber()
  {
    m_climberMotor = new CANSparkMax(RobotMap.climbMotor, MotorType.kBrushless);
    m_climbPID = m_climberMotor.getPIDController();
    m_climbEncoder = m_climberMotor.getEncoder();

    m_climbPID.setP(0.07);

    //m_frontSolenoid = new DoubleSolenoid(8, );

  }




  /**
   * function to extend the front pistons to climb
   */
  private void extendPiston()
  {
    m_frontSolenoid.set(Value.kForward);
  }


  /**
   * function to turn the neo motor to the desired level to climb
   * @param revs - the revolutions the motor has to turn
   */
  private void setMotor(double revs)
  {
    m_climbPID.setReference(revs, ControlType.kSmartMotion);
  }

  /**
   * function to climb to second level
   */
  public void secondLevel()
  {
    setMotor(climbLevel.kSecondLvl.getLevel());
    if((m_climbEncoder.getPosition() >= climbLevel.kSecondLvl.getLevel()- 0.6) && 
      (m_climbEncoder.getPosition() <= climbLevel.kSecondLvl.getLevel() + 0.4))
    {
      extendPiston();
      m_frontExtended = true;
    }
  }

  /**
   * function to climb to third level
   */
  public void thirdLevel()
  {
    setMotor(climbLevel.kThirdLvl.getLevel());
    if((m_climbEncoder.getPosition() >= climbLevel.kThirdLvl.getLevel()- 0.6) && 
      (m_climbEncoder.getPosition() <= climbLevel.kThirdLvl.getLevel() + 0.4))
    {
      extendPiston();
      m_frontExtended = true;
    }
  }


  /**
   * homes the motor to the 0 position
   */
  public void homeBase()
  {
    setMotor(0);
  }

  /** 
   * function to retract the front pistons to not climb
   */
  public void retractPiston()
  {
    m_frontSolenoid.set(Value.kReverse);
  }

  /**
   * function to reset the encoder to 0
   */
  public void resetEncoder()
  {
    m_climbEncoder.setPosition(0);
  }

  /**
   * function to actually climb
   */
  public void climb()
  {
    if(m_isSecond)
    {
      secondLevel();
    }else
    {
      thirdLevel();
    }
  }

  public void manualControl(double v, double vleft)
  {
    m_climbPID.setReference(v - vleft, ControlType.kDutyCycle);

  }

  public void testClimb()
  {
    m_climbPID.setReference(-789, ControlType.kPosition);
  }


  public void updateSmartDashboard()
  {
    SmartDashboard.putNumber("Climb Pos", m_climbEncoder.getPosition());
    SmartDashboard.putBoolean("Base at home", m_climbEncoder.getPosition() < 14);
    SmartDashboard.putBoolean("ClimbingSecondLvl", m_isSecond);

    m_isSecond = SmartDashboard.getBoolean("ClimbingSecondLvl", false);
    System.out.println(m_isSecond);
  }




  /**
   * static function to return the instance of the climber singleton
   * @return - the singleton instance
   */
  public static Climber getInstance()
  {
    return instance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //setDefaultCommand(new ClimberTestCommand());
  }
}
