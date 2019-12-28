/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.StateControl.ArmSetpoints;
import frc.robot.StateControl.IArmState;
import frc.robot.StateControl.ArmStates.BallGetterState;
import frc.robot.StateControl.ArmStates.CargoShipState;
import frc.robot.StateControl.ArmStates.HighGoalState;
import frc.robot.StateControl.ArmStates.LowGoalState;
import frc.robot.StateControl.ArmStates.MidGoalState;
import frc.robot.StateControl.ArmStates.NeutralState;
import frc.robot.commands.Arm.DefualtArmCommand;

/**
 * class for controlling the Arm, 
 * uses a Finate State Machine to control
 * the position of the arm
 */
public class Arm extends Subsystem {

  public enum TargetPos
  {
    kHatch, kCargo;
  }

  private static Arm m_armInstance = new Arm();

  private TalonSRX m_master, m_slave;

  private IArmState highGoalState;
  private IArmState midGoalState;
  private IArmState lowGoalState;
  private IArmState cargoShipState; 
  private IArmState neutralState;
  private IArmState ballGetterState;
  private IArmState state; 

  private TargetPos target = TargetPos.kCargo;
  private Arm()
  {
    //set up interfaces for state control
    highGoalState  = new HighGoalState(this);
    midGoalState   = new MidGoalState(this);
    lowGoalState   = new LowGoalState(this);
    cargoShipState = new CargoShipState(this);
    neutralState   = new NeutralState(this);
    ballGetterState = new BallGetterState(this);
    setState(ArmSetpoints.kBallGetter);

    //initialize arm motors
    m_master = new TalonSRX(RobotMap.armMaster);
    m_slave = new TalonSRX(RobotMap.armSlave);

    m_master.setInverted(InvertType.InvertMotorOutput);
    //configure slave
    m_slave.follow(m_master);
    m_slave.setInverted(InvertType.OpposeMaster);

    //config arm PID
    m_master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_master.setSensorPhase(false);
    m_master.configMotionCruiseVelocity(30000/2);
    m_master.configMotionAcceleration(20000/2);
    m_master.config_kF(0, Constants.arm_KF);
    m_master.config_kP(0, Constants.arm_KP);
    m_master.config_kD(0, Constants.arm_KD);

    m_master.configVoltageCompSaturation(12, 100);
    m_slave.configVoltageCompSaturation(12, 100);
    m_master.configContinuousCurrentLimit(40);
    m_slave.configContinuousCurrentLimit(40);
    m_master.configPeakCurrentLimit(38);
    m_slave.configPeakCurrentLimit(38);
    m_master.enableVoltageCompensation(true);
    m_slave.enableVoltageCompensation(true);

  }


  /**
   * function used for commands to run the arm to positions
   * dictated by the state interfaces
   */
  public void controlLoop()
  {
    moveArmToAngle();
    updateSmartDashboard();
  }



  /**
   * function that runs the motors in the arm to a desired position
   * @param tick - the tick positon for the encoder to reach
   */
  public void set(double tick)
  {
    m_master.set(ControlMode.MotionMagic, tick);
  }


  /**
   * function that changes the state the arm is in
   * @param setpoint - the desired state of the arm
   */
  public void setState(ArmSetpoints setpoint)
    {
        switch(setpoint)
        {
            case kHighGoal:
                state = highGoalState;
                break;
            case kMidGoal:
                state = midGoalState;
                break;
            case kLowGoal:
                state = lowGoalState;
                break;
            case kNeutral:
                state = neutralState;
                break;
            case kCargoShip:
                state = cargoShipState;
                break;
            case kBallGetter:
                state = ballGetterState;
                break;
        }
    }


  /**
   * function to change the target of the state
   * @param targetPos - the target for the arm to go to
   */
  public void setTarget(TargetPos targetPos)
  {
    switch(targetPos)
    {
      case kHatch:
        target = TargetPos.kHatch;
        break;
      case kCargo:
        target = TargetPos.kCargo;
        break;
    }
  }


  /**
   * function that resets the encoder value to 0
   */
  public void resetEncoder()
  {
    m_master.setSelectedSensorPosition(0);
  }


  /**
   * function that turns the angle for the arm into the ticks
   * for the encoder
   * @param angle - the desired angle of the arm
   * @return - ticks for the encoder
   */
  private double angleToTick(double angle)
  {
    return angle*Constants.anglePerTick;
  }


  /**
   * function that moves the arm to the angle given by the state machine
   */
  private void moveArmToAngle()
  {
    switch (target) {
      case kCargo:
        state.moveArmToBallAngle();
        break;
      case kHatch:
        state.moveArmToHatchAngle();
        break;
    }
    
  }

  /**
   * updates smart dashboard with the telemetry from the FSM
   */
  public void updateSmartDashboard()
  {
    state.updateSmartDashboard();
    SmartDashboard.putNumber(("Arm pos"), m_master.getSelectedSensorPosition());
    System.out.println("Arm" + m_master.getSelectedSensorPosition());
  }

  /**
   * function to raise the arm to a given height
   * @param height of person in inches
   */
  public void raiseToHeight(double height)
  {
    
  }



  /**
   * static factory for the arm singleton
   * @return - arm instance
   */
  public static Arm getInstance()
  {
    return m_armInstance;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DefualtArmCommand());
  }
}
