package team7111.lib.pathfinding;


/**
 * Class to hold waypoint's constraints. Contains a max speed, a min speed, and a tolerance.
 */
public class WaypointConstraints
{
    private double maxSpeed;
    private double minSpeed;
    /**In meters*/
    private double tolerance;

    /**
     * Constructs the waypoint parameter object.
     * <p> If used for translation, units are meters 
     * <p> If used for rotation, units are degrees
     * @param maxSpeed -Maximum speed in units per second.
     * @param minSpeed -Minimum speed in units per second the robot must be going during both travel and end of waypoint.
     * @param tolerance -Allowed distance from the waypoint the robot must be to be considered 
     */
    public WaypointConstraints(double maxSpeed, double minSpeed, double tolerance)
    {
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.tolerance = tolerance;
    }

    /**
     * Returns the tolerance the robot needs to be to the waypoint in units. <p>
     * Translation units: Meters. <p>
     * Rotation units: Degrees.
     */
    public double getTolerance()
    {
        return this.tolerance;
    }


    /**
     * Returns maximum allowed speed in units per second.
     */
    public double getMaxSpeed()
    {
        return this.maxSpeed;
    }

    /**
     * Returns minimum allowed speed in units per second.
     */
    public double getMinSpeed()
    {
        return this.minSpeed;
    }

};