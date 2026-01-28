package team7111.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.lib.pathfinding.Path;
import team7111.lib.pathfinding.Waypoint;
import team7111.lib.pathfinding.WaypointConstraints;
import team7111.robot.subsystems.SuperStructure.SuperState;
import team7111.robot.utils.AutoAction;

public class Autonomous extends SubsystemBase {

    private WaypointConstraints fastTransConstraints = new WaypointConstraints(8, 2, 0.5);
    private WaypointConstraints fastRotConstraints = new WaypointConstraints(720, 0, 5);
    
    private WaypointConstraints balancedTransConstraints = new WaypointConstraints(5, 0, 0.005);
    private WaypointConstraints balancedRotConstraints = new WaypointConstraints(720, 0, 1);

    private WaypointConstraints slowTransConstraints = new WaypointConstraints(2, 0, 0.001);
    private WaypointConstraints slowRotConstraints = new WaypointConstraints(180, 0, 0.5);

    public enum Autos {
        shootPreload,
        forwardTest,
        rotateTest,
    }

    public enum Paths {
        forward,
        rotate90
    }

    public Autonomous(){

    }

    public Waypoint fastPoint(double x, double y, double rotDegrees) {
       return new Waypoint(new Pose2d(x, y, Rotation2d.fromDegrees(rotDegrees)), fastTransConstraints, fastRotConstraints);
    }

    public Waypoint balancedPoint(double x, double y, double rotDegrees) {
       return new Waypoint(new Pose2d(x, y, Rotation2d.fromDegrees(rotDegrees)), balancedTransConstraints, balancedRotConstraints);
    }

    public Waypoint slowPoint(double x, double y, double rotDegrees) {
       return new Waypoint(new Pose2d(x, y, Rotation2d.fromDegrees(rotDegrees)), slowTransConstraints, slowRotConstraints);
    }

    public void periodic(){}

    public void simulationPeriodic(){}

    public List<AutoAction> getAutonomous(Autos autoName){
        ArrayList<AutoAction> auto = new ArrayList<>();
        // define each autonomous using a switch statement.
        // each auto is an array of "AutoAction's"
        switch (autoName) {
            case forwardTest:
                auto.add(new AutoAction(getPath(Paths.forward)));
                break;
            case rotateTest:
                auto.add(new AutoAction(getPath(Paths.rotate90)));
                break;
        
            default:

                break;
        }
        return auto;
    }

    public Path getPath(Paths path){
        List<Waypoint> waypoints = new ArrayList<>();
        // define Path object for each Paths enum using a switch statement
        switch (path) {
            case forward:
                waypoints.add(new Waypoint(new Pose2d(1, 0, Rotation2d.fromDegrees(0)), balancedTransConstraints, balancedRotConstraints));
                break;
            case rotate90:
                waypoints.add(new Waypoint(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), balancedTransConstraints, balancedRotConstraints));
                break;
        }
        return new Path(waypoints);
    }
}
