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
    kBallGetter(0,0),
    kCargoShip(218000/2,0),
    kLowGoal(106000/2,100000/2), //hello gamers
    kMidGoal(273000/2, 163500/2), //today we are going to learn how to hello
    kHighGoal(215000, 0);

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