
package frc.robot.StateControl;
public interface IArmState
{
    /**
     * moves the arm to the angle designated for the rocket cargo slot
     */
    void moveArmToBallAngle();

    /**
     * moves the arm to the angle designated for the rocket hatch slot
     */
    void moveArmToHatchAngle();

    /**
     * updates the telemetry on the SmartDashboard
     */
    void updateSmartDashboard();
}