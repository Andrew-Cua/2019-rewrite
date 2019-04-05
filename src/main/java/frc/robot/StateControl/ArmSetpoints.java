package frc.robot.StateControl;

/**
 * enumeration that holds the encoder tick positions for the arm setpoints
 * used for Finate State Machine that controls the arm
 */
public enum ArmSetpoints
{

    /**
     * (cargoTick, hatchTick)
     */
    kNeutral(0,0),
    kBallGetter(38000,380000),
    kCargoShip(218000,0),
    kLowGoal(106000,100000),
    kMidGoal(273000, 163500),
    kHighGoal(425000, 301453);

    private int cargoSetpoint;
    private int hatchSetpoint;
    private ArmSetpoints(int cargoSetpoint, int hatchSetpoint)
    {
        this.cargoSetpoint = cargoSetpoint;
        this.hatchSetpoint = hatchSetpoint;
    }

    public int getCargo()
    {
        return cargoSetpoint;
    }

    public int getHatch()
    {
        return hatchSetpoint;
    }
}