
package frc.robot.StateControl.WristStates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.StateControl.IWristState;
import frc.robot.StateControl.WristSetpoints;
import frc.robot.subsystems.Wrist;

/**
 * Add your docs here.
 */
public class LowGoalState implements IWristState
{
    private Wrist m_wrist;

    public LowGoalState(Wrist wrist)
    {
        m_wrist = wrist;
    }

    @Override
    public void moveWristToBallAngle() 
    {
        m_wrist.set(WristSetpoints.kLow.getCargo());
    }

    @Override
    public void moveWristToHatchAngle() 
    {
        m_wrist.set(WristSetpoints.kLow.getHatch());
    }

    @Override
    public void updateSmartDashboard() 
    {
        SmartDashboard.putNumber("Wrist position", 0);
    }
}