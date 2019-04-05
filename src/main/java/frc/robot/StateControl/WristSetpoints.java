package frc.robot.StateControl;

/**
 * enumeration that contains the tick pos for the various wrist pos
 */
public enum WristSetpoints
{
    /**
     * (cargo,hatch)
     */
    kNeutral(0,0),
    kBallGetter(6300,6300),
    kCargoShip(5900,0),
    kLow(3925,1925),
    kMid(4000, 1923),
    kHigh(4000,1609);

    private double cargoSetpoint;
    private double hatchSetpoint;
    private WristSetpoints(double cargoSetpoint, double hatchSetpoint)
    {
        this.cargoSetpoint = cargoSetpoint;
        this.hatchSetpoint = hatchSetpoint;
    }


    public double getCargo()
    {
        return cargoSetpoint;
    }

    public double getHatch()
    {
        return hatchSetpoint;
    }
}