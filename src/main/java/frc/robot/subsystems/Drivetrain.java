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

  private CANSparkMax m_leftMaster;
  private CANSparkMax m_leftSlaveA;
  private CANSparkMax m_leftSlaveB;

  private CANSparkMax m_rightMaster;
  private CANSparkMax m_rightSlaveA;
  private CANSparkMax m_rightSlaveB;

  private CANPIDController m_leftPID;
  private CANPIDController m_rightPID;

  private CANEncoder m_leftEncoder;
  private CANEncoder m_rightEncoder;
  private Drivetrain()
  {
    //initialize left motors
    m_leftMaster = new CANSparkMax(RobotMap.leftMaster, MotorType.kBrushless);
    m_leftSlaveA = new CANSparkMax(RobotMap.leftSlaveA, MotorType.kBrushless);
    m_leftSlaveB = new CANSparkMax(RobotMap.leftSlaveB, MotorType.kBrushless);

    //initialize right motors
    m_rightMaster = new CANSparkMax(RobotMap.rightMaster, MotorType.kBrushless);
    m_rightSlaveA = new CANSparkMax(RobotMap.rightSlaveA, MotorType.kBrushless);
    m_rightSlaveB = new CANSparkMax(RobotMap.rightSlaveB, MotorType.kBrushless);

    //initialize left PIDController and encoder
    m_leftPID = m_leftMaster.getPIDController();
    m_leftEncoder = m_leftMaster.getEncoder();

    //initialize right PIDController and encoder
    m_rightPID = m_rightMaster.getPIDController();
    m_rightEncoder = m_rightMaster.getEncoder();

    //invert rightside motors so that it can drive straight
    m_rightMaster.setInverted(true);
    m_rightSlaveA.setInverted(true);
    m_rightSlaveB.setInverted(true);

    //set slaves to follow masters
    m_leftSlaveA.follow(m_leftMaster);
    m_leftSlaveB.follow(m_leftMaster);

    //set slaves to follow masters
    m_rightSlaveA.follow(m_rightMaster);
    m_rightSlaveB.follow(m_rightMaster);

    //set left PID
    m_leftPID.setP(Constants.kP);
    m_leftPID.setI(Constants.kI);
    m_leftPID.setD(Constants.kD);
    m_leftPID.setSmartMotionMaxVelocity(Constants.kMAX_VEL, 0);
    m_leftPID.setSmartMotionMaxAccel(Constants.kMAX_ACCEL, 0);
    m_leftPID.setOutputRange(Constants.kOUTPUT_MIN, Constants.kOUTPUT_MAX);

    //set right PID
    m_rightPID.setP(Constants.kP);
    m_rightPID.setI(Constants.kI);
    m_rightPID.setD(Constants.kD);
    m_rightPID.setSmartMotionMaxVelocity(Constants.kMAX_VEL, 0);
    m_rightPID.setSmartMotionMaxAccel(Constants.kMAX_ACCEL, 0);
    m_rightPID.setOutputRange(Constants.kOUTPUT_MIN, Constants.kOUTPUT_MAX);

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

    double yValPrime = Math.pow((sensFactor*y), 3) + ((1-sensFactor)*y);
    double xValPrime = Math.pow((sensFactor*x), 3) + ((1-sensFactor)*x);
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
   * resets the encoder position by setting the current position to 0
   */
  public void resetEncoder()
  {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }


  /**
   * function that determines whether or not the robot
   * has moved the distance set by the moveDistance function
   * @param - the setpoint for the robot to move to
   * @return - true or false depending on where the robot is
   */
  public boolean onTarget(double inches)
  {
    double moveAverage = (m_leftEncoder.getPosition() + m_rightEncoder.getPosition())/2;
    double deadband = 0.25;
    if(moveAverage >= Constants.inchesToRev(inches - deadband) && moveAverage <= Constants.inchesToRev(inches + deadband))
    {
      return true;
    }
    return false;
  }


  public void updateSmartDashboard()
  {
    SmartDashboard.putNumber("Speed", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("Left Encoder", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Encoder", m_leftEncoder.getPosition());
  }


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
    m_leftPID.setReference(leftPower, type);
    m_rightPID.setReference(rightPower,type);

    //System.out.println(leftPower);
  }

  public void run()
  {
    m_leftMaster.set(1);
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
