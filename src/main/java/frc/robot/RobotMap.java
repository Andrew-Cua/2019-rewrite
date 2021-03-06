/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static int leftMaster = 11,
                    leftSlaveA = 14,
                    leftSlaveB = 14;

  public static int rightMaster = 13,
                    rightSlaveA = 15,
                    rightSlaveB = 15;

  public static int armMaster = 10,
                    armSlave = 11;
  public static int wristMotor = 13;

  public static int climbMotor = 20;
}
