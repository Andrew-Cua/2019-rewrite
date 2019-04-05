
package frc.robot.StateControl.ArmStates;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.StateControl.ArmSetpoints;
import frc.robot.StateControl.IArmState;
import frc.robot.subsystems.Arm;

/**
 * state for the high goal on the rocket
 * 
 * see IArmState.java for documentation
 */
public class LowGoalState implements IArmState
{

    private Arm m_arm;
    public LowGoalState(Arm arm)
    {
        m_arm = arm;
    }
    @Override
    public void moveArmToBallAngle() 
    {
        m_arm.set(ArmSetpoints.kLowGoal.getCargo());
    }

    @Override
    public void moveArmToHatchAngle() 
    {
        m_arm.set(ArmSetpoints.kLowGoal.getHatch());
    }

    @Override
    public void updateSmartDashboard() 
    {
        SmartDashboard.putString("Arm Position", "Low Goal");
    }

}
