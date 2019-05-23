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

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Drivetrain.DefaultDriveCommand;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  public static Drivetrain m_drivetrain = new Drivetrain();


  private VictorSPX m_leftMaster;
  private VictorSPX m_leftSlave;


  private VictorSPX m_rightMaster;
  private VictorSPX m_rightSlave;

  private Drivetrain()
  {
    //initialize left motors
    m_leftMaster = new VictorSPX(RobotMap.leftMaster);
    m_leftSlave  = new VictorSPX(RobotMap.leftSlaveA);


    m_rightMaster = new VictorSPX(RobotMap.rightMaster);
    m_rightSlave  = new VictorSPX(RobotMap.rightSlaveA);


    //invert rightside motors so that it can drive straight
    m_rightMaster.setInverted(InvertType.InvertMotorOutput);
    m_rightSlave.setInverted(InvertType.InvertMotorOutput);
    m_leftSlave.follow(m_leftMaster);

    //set slaves to follow masters
    m_rightSlave.follow(m_rightMaster);

      }

  /**
   * common function to drive the robot
   * uses velocity control for smoother driving
   * @param x - the x offset of the joystick from -1 to 1
   * @param y - the y offset of the joystick from -1 to 1
   */
  public void cartersianDrive(double x, double y)
  {
    if(Math.abs(x) < 0.1) x = 0;
    if(Math.abs(y) < 0.1) y = 0;

    double sensFactor = 0.75D;

    double yValPrime = Math.pow((sensFactor*y), 5) + ((1-sensFactor)*y);
    double xValPrime = Math.pow((sensFactor*x), 5) + ((1-sensFactor)*x);
    double leftPower  =  yValPrime - xValPrime;
    double rightPower =  yValPrime + xValPrime;

    double leftVelocity  = (y - x);// * Constants.kMAX_VEL;
    double rightVelocity = (y + x);// * Constants.kMAX_VEL;
    set(leftPower, rightPower, ControlType.kDutyCycle);
  }

  /**
  * function that moves to robot a set number of inches
  * @param inches
  */
  public void moveDistance(double inches)
  {
    //resetEncoder();
    SmartDashboard.putNumber("Rotations", Constants.inchesToRev(inches));
    double leftDistance  = Constants.inchesToRev(inches);
    double rightDistance = Constants.inchesToRev(inches); 
    
    set(leftDistance, rightDistance, ControlType.kPosition);
  }
  //haha epic comment






  /**
   * function that manuvers the robot towards the target tracked by 
   * the current pipeline
   */
  public void seekTarget()
  {
    double kP = 0.0052;
    double x  = Robot.m_limelight.getX();
    double y  = Robot.m_limelight.getY();
    boolean v = Robot.m_limelight.getV() != 0;
    
    double power = -0.20;
    if(!v){return;}

    if(v)
    {
      double headingAdjust = x*kP;
      double leftPower = power - headingAdjust;
      double rightPower = power + headingAdjust;

      set(leftPower, rightPower, ControlType.kDutyCycle);
    }
  }


  /**
   * function that sets the power of the motors in the drivetrain 
   * behavior changes based on the control type
   * for the most part use SmartVelocity during teleop control and SmartMotion for auto
   * @param leftPower - power given to the leftmotors (changes based on control type)
   * @param rightPower - power given to the right (changes based on control type)
   * @param type
   */
  private void set(double leftPower, double rightPower, ControlType type)
  {
    m_leftMaster.set(ControlMode.PercentOutput,leftPower);
    m_rightMaster.set(ControlMode.PercentOutput,rightPower); 
  }






  public static Drivetrain getInstance()
  {
    return m_drivetrain;
  }
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DefaultDriveCommand());
  }
}
