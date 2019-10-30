/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.StateControl.IWristState;
import frc.robot.StateControl.WristSetpoints;
import frc.robot.StateControl.WristStates.NeutralState;
import frc.robot.StateControl.WristStates.MidGoalState;
import frc.robot.StateControl.WristStates.LowGoalState;
import frc.robot.StateControl.WristStates.BallGetterState;
import frc.robot.StateControl.WristStates.CargoShipState;
import frc.robot.StateControl.WristStates.HighGoalState;
import frc.robot.StateControl.WristStates.TeleopState;
import frc.robot.commands.Wrist.DefualtWristCommand;

/**
 * class that controls the movement and position of the wrist
 */
public class Wrist extends Subsystem 
{
  private static Wrist wrist = new Wrist();
  private TalonSRX m_wristMotor;
  public enum WristControlType
  {kTeleop, kAutonomous;}
  public enum TargetPos
  {kCargo, kHatch;}

  private WristSetpoints m_desiredSetpoint = WristSetpoints.kCargoShip;
  private WristControlType m_control = WristControlType.kTeleop;
  private TargetPos m_target = TargetPos.kCargo;

  private IWristState teleopState;
  private IWristState cargoShipState;
  private IWristState lowGoalState;
  private IWristState midGoalState;
  private IWristState highGoalState;
  private IWristState neutralState;
  private IWristState ballGetterState;
  private IWristState state;

  private Wrist()
  {
    //initialize wrist motor
    m_wristMotor = new TalonSRX(RobotMap.wristMotor);

    //configure motion magic
    m_wristMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_wristMotor.configMotionCruiseVelocity(1000);
    m_wristMotor.configMotionAcceleration(830);
    m_wristMotor.setSensorPhase(true);
    //config PID
    m_wristMotor.config_kF(0,1.1443);
    m_wristMotor.config_kP(0,1.1705);

    //initialize state control 
    teleopState    = new TeleopState(this);
    cargoShipState = new CargoShipState(this);
    lowGoalState   = new LowGoalState(this);
    midGoalState   = new MidGoalState(this);
    highGoalState  = new HighGoalState(this);
    neutralState   = new NeutralState(this);
    ballGetterState= new BallGetterState(this);
    state = neutralState;

  }


  /**
   * called in the default command for the wrist
   * it determines what mode the wrist is in and its target
   * and moves the wrist to the desired position 
   */
  public void controlLoop()
  {
    if(m_control == WristControlType.kTeleop)
    {
      SmartDashboard.putString("Control Mode", "Manual");
      set(((Robot.m_oi.getIntakeStick().getRawAxis(3)+1)/2)*9000);
    }else if(m_control == WristControlType.kAutonomous)
    {
      switch (m_target) {
        case kCargo:
            moveWristToCargoAngle();
          break;
        case kHatch:
            moveWristToHatchAngle();
          break;
      }
      SmartDashboard.putString("Control Mode", "Auto");
    }
    updateSmartDashboard();
  }


  /**
   * sets the position for the wrist to move to
   * @param angle - the desired angle of the wrist
   */
  public void set(double angle)
  {
    m_wristMotor.set(ControlMode.MotionMagic, angle);
  }


  /**
   * resets the encoder by setting the position to 0
   */
  public void resetEncoder()
  {
    m_wristMotor.setSelectedSensorPosition(0);
  }


  /**
   * function that toggles the control mode of the wrist
   */
  public void toggleControlMode()
  {
    switch (m_control) {
      case kTeleop:
          updateControlMode(WristControlType.kAutonomous);
        break;
      case kAutonomous:
          updateControlMode(WristControlType.kTeleop);
        break;
    }
  }

  /**
   * sets the state of the arm FSM, not sure how reliable,
   * never been used
   * @param setpoint
   */
  public void setState(WristSetpoints setpoint)
  {
      switch(setpoint)
      {
          case kHigh:
              state = highGoalState;
              break;
          case kMid:
              state = midGoalState;
              break;
          case kLow:
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
   * sets the target for the wrist to go to depending 
   * on whether it needs to aim for cargo or hatches
   * @param pos
   */
  public void setTarget(TargetPos pos)
  {
    m_target = pos;
  }


  /**
   * returns the desired setpoint of the wrist
   * @return - returns an instance of WristSetpoint
   */
  public WristSetpoints getDesiredSetpoint()
  {
    return m_desiredSetpoint;
  }


  /**
   * changes the control type of the wrist,
   * from user controlled to autnomous 
   * @param type - the control type of the wrist
   */
  private void updateControlMode(WristControlType type)
  {
    m_control = type;
  }

  /**
   * computes the ticks needed for the wrist from a given angle
   * @param angle - the angle the wrist needs to be at
   * @return - the ticks for the encoder on the wrist
   */
  private double angleToTick(double angle)
  {
    return angle*(180/6700);
  }



  /**
   * function that moves the wrist to the position 
   * dictated by the current state for cargo
   */
  private void moveWristToCargoAngle()
  {
    state.moveWristToBallAngle();
  }


  /**
   * function that moves the wrist to the position
   * dictated by the current state for hatch panels
   */
  private void moveWristToHatchAngle()
  {
    state.moveWristToHatchAngle();
  }

  /**
   * function that updates the dashboard with telemetry
   * from the wrist
   */
  public void updateSmartDashboard()
  {
    state.updateSmartDashboard();
    SmartDashboard.putNumber("WristPos", m_wristMotor.getSelectedSensorPosition());
    SmartDashboard.putBoolean("User Controlled", ((m_control == WristControlType.kTeleop)? true: false));
    
  }



  /**
   * static function for the wrist singleton
   * @return - returns the premade instance of the wrist class
   */
  public static Wrist getInstance()
  {
    return wrist;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DefualtWristCommand());
  }
}
