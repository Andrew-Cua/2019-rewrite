/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.StateControl.WristStates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.StateControl.IWristState;
import frc.robot.subsystems.Wrist;

/**
 * Add your docs here.
 */
public class TeleopState implements IWristState
{
    private Wrist m_wrist;

    public TeleopState(Wrist wrist)
    {
        m_wrist = wrist;
    }

    @Override
    public void moveWristToBallAngle() 
    {
        m_wrist.set(0);
    }

    @Override
    public void moveWristToHatchAngle() 
    {
        m_wrist.set(0);
    }

    @Override
    public void updateSmartDashboard() 
    {
        SmartDashboard.putNumber("Wrist position", 0);
    }
}
