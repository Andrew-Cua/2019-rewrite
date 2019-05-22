
package frc.robot.StateControl.WristStates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.StateControl.IWristState;
import frc.robot.StateControl.WristSetpoints;
import frc.robot.subsystems.Wrist;

/**
 * Add your docs here.
 */
public class BallGetterState implements IWristState
{
    private Wrist m_wrist;

    public BallGetterState(Wrist wrist)
    {
        m_wrist = wrist;
    }

    @Override
    public void moveWristToBallAngle() 
    {
        m_wrist.set(WristSetpoints.kBallGetter.getCargo());
    }

    @Override
    public void moveWristToHatchAngle() 
    {
        m_wrist.set(WristSetpoints.kBallGetter.getHatch());
    }

    @Override
    public void updateSmartDashboard() 
    {
        SmartDashboard.putNumber("Wrist position", 0);
    }
}