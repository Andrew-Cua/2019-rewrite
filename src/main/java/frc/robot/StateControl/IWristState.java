
package frc.robot.StateControl;
public interface IWristState
{
    /**
     * moves the wrist to the angle designated for the rocket cargo slot
     */
    void moveWristToBallAngle();

    /**
     * moves the arm to the angle designated for the rocket hatch slot
     */
    void moveWristToHatchAngle();

    /**
     * updates the telemetry on the SmartDashboard
     */
    void updateSmartDashboard();
}