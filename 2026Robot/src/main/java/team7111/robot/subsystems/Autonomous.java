package team7111.robot.subsystems;

import java.util.ArrayList;
import java.util.Collection;
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
    private SuperStructure superStructure;

    private final Waypoint[] trenchLWaypoints = new Waypoint[]{
        new Waypoint(new Pose2d(3.8, 7.446, Rotation2d.fromDegrees(0)), balancedTransConstraints, balancedRotConstraints),
        new Waypoint(new Pose2d(5.9, 7.446, Rotation2d.fromDegrees(0)), balancedTransConstraints, balancedRotConstraints),
    };
    private final Waypoint[] trenchRWaypoints = new Waypoint[]{
        new Waypoint(new Pose2d(3.8, 0.675, Rotation2d.fromDegrees(0)), balancedTransConstraints, balancedRotConstraints),
        new Waypoint(new Pose2d(5.9, 0.675, Rotation2d.fromDegrees(0)), balancedTransConstraints, balancedRotConstraints),
    };

    public enum Autos {
        forwardTest,
        leftNeutral,
    }

    public enum Paths {
        forward,

        hubSetpointL,
        hubSetpointM,
        hubSetpointR,
        trenchLNeutral,
        trenchRNeutral,
        trenchLAlliance,
        trenchRAlliance,
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

            case leftNeutral:
                auto.add(new AutoAction(SuperState.prepareHubShot).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.hubSetpointL)));
                //auto.add(new AutoAction(SuperState.prepareHubShot));
                //auto.add(new AutoAction(SuperState.score));
                //auto.add(new AutoAction(SuperState.intake));
                auto.add(new AutoAction(getPath(Paths.trenchLNeutral)));
                auto.add(new AutoAction(getPath(Paths.trenchLAlliance)));
                auto.add(new AutoAction(SuperState.prepareHubShot).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.hubSetpointL)));
                auto.add(new AutoAction(SuperState.score));
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
            
            case hubSetpointL:
                waypoints.add(balancedPoint(2.266, 5.946, -30.65));
                break;
            case hubSetpointM:
                waypoints.add(balancedPoint(2.267, 4.021, 0));
                break;
            case hubSetpointR:
                waypoints.add(balancedPoint(1.975, 1.986, 38.21));
                break;
            case trenchLAlliance:
                waypoints.add(trenchLWaypoints[1]);
                waypoints.add(trenchLWaypoints[0]);
                break;
            case trenchLNeutral:
                waypoints.add(trenchLWaypoints[0]);
                waypoints.add(trenchLWaypoints[1]);
                break;
            case trenchRAlliance:
                waypoints.add(trenchRWaypoints[1]);
                waypoints.add(trenchRWaypoints[0]);
                break;
            case trenchRNeutral:
                waypoints.add(trenchRWaypoints[0]);
                waypoints.add(trenchRWaypoints[1]);
                break;
            default:
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

    public Path getNearestHubScoringPath(Pose2d robotPose){
        List<Path> hubPaths = new ArrayList<>();
        List<Pose2d> hubPoses = new ArrayList<>();
        hubPaths.add(getPath(Paths.hubSetpointL));
        hubPaths.add(getPath(Paths.hubSetpointM));
        hubPaths.add(getPath(Paths.hubSetpointR));

        for (Path path : hubPaths) {
            hubPoses.add(path.getCurrentWaypoint().getPose());
        }

        Pose2d pose = robotPose.nearest(hubPoses);

        for (Path path : hubPaths) {
            if(pose == path.getCurrentWaypoint().getPose()){
                return path;
            }
        }
        return null;
    }

    public void giveResources(SuperStructure superStructure){
        this.superStructure = superStructure;
    }
}
