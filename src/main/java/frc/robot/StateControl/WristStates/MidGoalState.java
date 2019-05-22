
package frc.robot.StateControl.WristStates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.StateControl.IWristState;
import frc.robot.StateControl.WristSetpoints;
import frc.robot.subsystems.Wrist;

/**
 * Add your docs here.
 */
public class MidGoalState implements IWristState
{
    private Wrist m_wrist;

    public MidGoalState(Wrist wrist)
    {
        m_wrist = wrist;
    }

    @Override
    public void moveWristToBallAngle() 
    {
        m_wrist.set(WristSetpoints.kMid.getCargo());
    }

    @Override
    public void moveWristToHatchAngle() 
    {
        m_wrist.set(WristSetpoints.kMid.getHatch());
    }

    @Override
    public void updateSmartDashboard() 
    {
        SmartDashboard.putNumber("Wrist position", 0);
    }
}