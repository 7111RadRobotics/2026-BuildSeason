package team7111.robot.subsystems;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.lib.pathfinding.Path;
import team7111.lib.pathfinding.Waypoint;
import team7111.lib.pathfinding.WaypointConstraints;
import team7111.robot.subsystems.SuperStructure.SuperState;
import team7111.robot.utils.AutoAction;

public class Autonomous extends SubsystemBase {

    Timer timer = new Timer();

    Zones zone;


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

    public final Pose2d[] hubPresetPoses = new Pose2d[]{
        // TODO get coordinates for poses near trench
        new Pose2d(4.25, 0.625, Rotation2d.fromDegrees(90)),
        new Pose2d(4.25, 7.45, Rotation2d.fromDegrees(-90)),
        new Pose2d(2.266, 5.946, Rotation2d.fromDegrees(-30.65)),
        new Pose2d(2.267, 4.021, Rotation2d.fromDegrees(0)),
        new Pose2d(1.975, 1.986, Rotation2d.fromDegrees(38.21)),
    };
    //R = Right, L = Left, I = Intake, N = Neutral zone, A = alliance zone
     
    public enum Autos {
        forwardTest,
        leftNeutral,
        Rhub2IN,
        Lhub2IN,
    }

    public enum Paths {
        forward,

        hubSetpointL,
        hubSetpointM,
        hubSetpointR,
        hubSetpointRT,
        hubSetpointLT,
        trenchLNeutral,
        trenchRNeutral,
        trenchLAlliance,
        trenchRAlliance,

        RNsweep,
        LNsweep,
    }

    public Autonomous(Zones zone){
        for (Autos auto : Autos.values()) {
            autoChooser.addOption(auto.name(), auto);
        }
        this.zone = zone;
        
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
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score));
                //auto.add(new AutoAction(SuperState.intake));
                auto.add(new AutoAction(getPath(Paths.trenchLNeutral)));
                auto.add(new AutoAction(getPath(Paths.trenchLAlliance)));
                auto.add(new AutoAction(SuperState.prepareHubShot).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.hubSetpointL)));
                auto.add(new AutoAction(SuperState.score));
                break;

            case Rhub2IN:
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> timeDelay(5)));
                auto.add(new AutoAction(SuperState.intake).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.RNsweep)));
                auto.add(new AutoAction(SuperState.deployed).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.hubSetpointLT)));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> timeDelay(5)));
                break;
            
            case Lhub2IN:
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> timeDelay(5)));
                auto.add(new AutoAction(SuperState.intake).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.LNsweep)));
                auto.add(new AutoAction(SuperState.deployed).withNoConditions());
                auto.add(new AutoAction(getPath(Paths.hubSetpointRT)));
                auto.add(new AutoAction(SuperState.prepareHubShot));
                auto.add(new AutoAction(SuperState.score).withAlternateCondition(() -> timeDelay(5)));
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
            case hubSetpointRT:
                waypoints.add(balancedPoint(4.25, 0.625, 90));
                break;
            case hubSetpointLT:
                waypoints.add(balancedPoint(4.25, 7.45, -90));
                break;
            case hubSetpointL:
                waypoints.add(balancedPoint(hubPresetPoses[2].getX(), hubPresetPoses[2].getY(), hubPresetPoses[2].getRotation().getDegrees()));
                break;
            case hubSetpointM:
                waypoints.add(balancedPoint(hubPresetPoses[3].getX(), hubPresetPoses[3].getY(), hubPresetPoses[3].getRotation().getDegrees()));
                break;
            case hubSetpointR:
                waypoints.add(balancedPoint(hubPresetPoses[4].getX(), hubPresetPoses[4].getY(), hubPresetPoses[4].getRotation().getDegrees()));
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
            case RNsweep:
                waypoints.add(balancedPoint(8.182, 0.875, 90));
                waypoints.add(balancedPoint(8.2, 7.3, 0));
                //waypoints.add(balancedPoint(4.27, 7.484, -90));
                break;
            case LNsweep:
                waypoints.add(balancedPoint(8.2, 7.3, -90));
                waypoints.add(balancedPoint(8.2, 0.875, 180));
                //waypoints.add(balancedPoint(4.217, 0.521, 90));
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
        //TODO this function will return the nearest Pose2d to robotPose from hubPresetPoses on line 45.
        // If it is one of the trench poses (one of the first 2 poses in the array), it will check if it is closest to
        // that same pose, the pose with 1.5 x added, or the pose with 1.5 x subtracted.
        // Use the nearest() method in the Pose2d class to find the nearest pose.
        List<Waypoint> waypoints = new ArrayList<>();
        List<Pose2d> hubPoses = new ArrayList<>();
        if (zone.inAllianceZone(robotPose)) {
            for (Pose2d pose: hubPresetPoses) {
                hubPoses.add(pose);
            }
        } else {
            hubPoses.add(hubPresetPoses[0]);
            hubPoses.add(hubPresetPoses[1]);
        }
        
        Double robotPoseX = robotPose.nearest(hubPoses).getX();
        Double robotPoseY = robotPose.nearest(hubPoses).getY();
        Double robotPoseRot = robotPose.nearest(hubPoses).getRotation().getDegrees();

        if (robotPoseX == hubPresetPoses[1].getX() && robotPoseY == hubPresetPoses[1].getY()) {
            if (zone.inAllianceZone(robotPose)) {
                waypoints.add(balancedPoint(robotPoseX - 1, robotPoseY, robotPoseRot));
            } else {
                waypoints.add(balancedPoint(robotPoseX + 1.5, robotPoseY, robotPoseRot));
            }
        } else if (robotPoseX == hubPresetPoses[0].getX() && robotPoseY == hubPresetPoses[0].getY()) {
            if (zone.inAllianceZone(robotPose)) {
                waypoints.add(balancedPoint(robotPoseX - 1, robotPoseY, robotPoseRot));
            } else {
                waypoints.add(balancedPoint(robotPoseX + 1.5, robotPoseY, robotPoseRot));
            }
        }
        waypoints.add(new Waypoint(robotPose.nearest(hubPoses), balancedTransConstraints, balancedRotConstraints));
        return new Path(waypoints);
    }

    public Path getNearestTrenchPath(Pose2d robotPose){
        List<Path> trenchPaths = new ArrayList<>();
        List<Pose2d> trenchPoses = new ArrayList<>();
        if(robotPose.getX() > 4.7 && robotPose.getX() < 11.957){
            trenchPaths.add(getPath(Paths.trenchLAlliance));
            trenchPaths.add(getPath(Paths.trenchRAlliance));
        }else{
            trenchPaths.add(getPath(Paths.trenchLNeutral));
            trenchPaths.add(getPath(Paths.trenchRNeutral));
        }

        for (Path path : trenchPaths) {
            trenchPoses.add(path.getCurrentWaypoint().getPose());
        }

        Pose2d pose = robotPose.nearest(trenchPoses);

        for (Path path : trenchPaths) {
            if(pose.getX() == path.getCurrentWaypoint().getPose().getX()
             && pose.getY() == path.getCurrentWaypoint().getPose().getY()){
                return path;
            }
        }
        return null;
    }

    public void giveResources(SuperStructure superStructure){
        this.superStructure = superStructure;
    }

    private boolean timeDelay(int delay){
        timer.start();
        if (timer.hasElapsed(delay)) {
            timer.reset();
            timer.stop();
            return true;
        }
        return false;
    }
}
