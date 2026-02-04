package team7111.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.lib.pathfinding.Path;
import team7111.lib.pathfinding.Waypoint;
import team7111.lib.pathfinding.WaypointConstraints;
import team7111.robot.subsystems.SuperStructure.SuperState;
import team7111.robot.utils.AutoAction;

public class Autonomous extends SubsystemBase {

    private WaypointConstraints fastTransConstraints = new WaypointConstraints(8, 2, 0.5);
    private WaypointConstraints fastRotConstraints = new WaypointConstraints(720, 0, 90);
    
    private WaypointConstraints balancedTransConstraints = new WaypointConstraints(5, 0, 0.1);
    private WaypointConstraints balancedRotConstraints = new WaypointConstraints(720, 0, 1);

    private WaypointConstraints slowTransConstraints = new WaypointConstraints(2, 0, 0.002);
    private WaypointConstraints slowRotConstraints = new WaypointConstraints(180, 0, 0.3);

    private SendableChooser<Autos> autoChooser = new SendableChooser<>();

    public enum Autos {
        shootPreload,
        forwardTest,
        rotateTest,
        multiPoint,
        forward180,
    }

    public enum Paths {
        forward,
        rotate90,
        forward2,
        forwardRotate180
    }

    public Autonomous(){
        for (Autos auto : Autos.values()) {
            autoChooser.addOption(auto.name(), auto);
        }
        
        Shuffleboard.getTab("Autonomous").add("AutoChooser", autoChooser);
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
            case multiPoint:
                auto.add(new AutoAction(getPath(Paths.forward)));
                auto.add(new AutoAction(SuperState.temp1));
                auto.add(new AutoAction(getPath(Paths.forward2)));
                break;
            case forward180:
                auto.add(new AutoAction(getPath(Paths.forwardRotate180)));
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
                waypoints.add(balancedPoint(1, 0, 0));
                break;
            case rotate90:
                waypoints.add(balancedPoint(0, 0, 90));
                break;
            case forward2:
                waypoints.add(slowPoint(2, 0, 90));
                break;
            case forwardRotate180:
                waypoints.add(balancedPoint(1, 0, 90));
                waypoints.add(balancedPoint(2, 0, 180));
                break;
        }
        return new Path(waypoints);
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

    public Autos getSelectedAuto(){
        return autoChooser.getSelected();
    }
}
