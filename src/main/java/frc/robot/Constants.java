/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants 
{
    public static final double kHIGH_GEAR_RATIO = 9.74;
    //in inches
    public static final double kWHEEL_RADIUS = 3;
    public static final double kWHEEL_CIRCUMFERENCE = Math.PI*(2*kWHEEL_RADIUS);
    public static final double kINCHES_PER_REV = kWHEEL_CIRCUMFERENCE/kHIGH_GEAR_RATIO;

    public static final double kQuickStopAlpha = 0.1;
    public static final double kDriveDeadband = 0.02;

    /**
     * turns inches into the amount of revs required by the motor
     * @param inches - distance to travel
     * @return revolutions for the motor
     */
    public static double inchesToRev(double inches)
    {
        return inches/kINCHES_PER_REV;
    }

    /**
     * tunrs the amounts of revolutions into the inches traveled 
     * @param rev - amount of revolutions counted from the encoder
     * @return - inches given by the revoutions traveled 
     */
    public static double revolutionsToInches(double rev)
    {
        return kINCHES_PER_REV*rev;
    }

    public static double heightToTicks(double height)
    {
        
        return height;
    }

    public static double applyDeadband(double x, double deadBand)
    {
        if(Math.abs(x) < deadBand)
        {
            return 0;
        }
        return x;
    }

    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
      }

    //drivetrain PID constants
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;

    //drivetrain motion constants
    public static final int kMAX_VEL = -5200;
    public static final int kMAX_ACCEL = -4000;
    public static final double kOUTPUT_MAX = 1.0;
    public static final double kOUTPUT_MIN = -1.0;


    //arm constants
    public static final double anglePerTick = 6160/135;
    //arm PID
    public static final double arm_KF = 0.033,
                               arm_KP = 0.033,
                               arm_KD = 0.00037;
}
