package team7111.lib.pathfinding;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Path {

    private double mapLengthX = 17.55; // width of field in meters
    private double mapLengthY = 8.05; // length of field in meters

    private boolean isflipped = false;

    //Robot position as a supplied Pose2d
    private Supplier<Pose2d> robotPose;

    //Translation speed of the robot in each axis, and rotation.
    private DoubleSupplier xTransSpeed = () -> 0.0;
    private DoubleSupplier yTransSpeed = () -> 0.0;
    private DoubleSupplier rotSpeed = () -> 0.0;

    private int currentWaypointIndex = 0;
    
    private Waypoint[] waypoints;
    private boolean isPathFinished = false;

    Translation2d waypointPos = null;
    Translation2d currentPos = null;
    double currentDistance = 0;
    double fieldLength = 0;
    double fieldWidth = 0;
    final double gridSize = 0.25;
    final double robotRadius = 0.4;
    private PosePlanning planner = new PosePlanning();
    double[][] directions = {
        {1,0},
        {0,1},
        {-1,0},
        {0,-1},
        {1,1},
        {-1,1},
        {1,-1},
        {-1,-1},
    };
    double currentX = 0;
    double currentY = 0;
    double pathWeight = 0;
    Map<Translation2d, Boolean> neighborStatus = new HashMap<>();
    Map<Translation2d, Double> fScore = new HashMap<>();
    private double startDistance = 0;
    private Translation2d initialPosition = null;
    private Optional<Map.Entry<Translation2d, Double>> minScore = Optional.empty();
    private Translation2d endPath = null;
    private Double endPathX = 7.2;
    private Double endPathY = 5.3;
    private Waypoint[] nodeWaypoints;
    public Waypoint tempWaypoint[] = new Waypoint[]{new Waypoint(new Pose2d(4, 4, Rotation2d.fromDegrees(180.0)), new WaypointConstraints(10, 0, 0.25), new WaypointConstraints(360, 0, 10))};

    /**
     * Constructs a path from several waypoints. Uses pathMaster class to define parameters.
     * @param waypoints -An array of waypoint objects.
     * @param robotPose -A supplier to be assigned to the local variable, letting path know robot pose.
     */
    public Path(Waypoint[] waypoints){
        this.waypoints = waypoints;

        var field = new Field2d();
        int index = 0;
        for(Waypoint waypoint : waypoints){
            field.getObject("Waypoint " + index).setPose(waypoint.getPose());
            index ++;
        }
        SmartDashboard.putData("Path Waypoints", field);
    }

    /**
     * Returns the current waypoint the robot is pathing to.
     * <p>Note current waypoint index is one ahead of array for waypoint.
     */
    public int getCurrentWaypointIndex(){
        return currentWaypointIndex;
    }

    /**
     * Returns the current waypoint object that the robot is pathing to.
     */
    public Waypoint getCurrentWaypoint(){
        return waypoints[currentWaypointIndex];
    }

    public Waypoint[] getWaypoints(){
        return waypoints;
    }
    
    /**
     * Returns in degrees per second, positive clockwise.
     */
    public double getRotationSpeed(){
        double speed = rotSpeed.getAsDouble();
        WaypointConstraints waypointConstraints = getCurrentWaypoint().getRotationConstraints();
        if(speed > waypointConstraints.getMaxSpeed()) {
            speed = waypointConstraints.getMaxSpeed();
        }
        if(speed < -waypointConstraints.getMaxSpeed()){
            speed = -waypointConstraints.getMaxSpeed();
        }
        if(Math.abs(speed) < waypointConstraints.getMinSpeed()){
            if(speed > 0){
                speed = waypointConstraints.getMinSpeed();
            }else{
                speed = -waypointConstraints.getMinSpeed();
            }
        }
        //speed = rotSpeed.getAsDouble();
        return speed;
    }

    /**
     * Returns in meters per second.
     */
    public double getTranslationXSpeed(){
        double combinedSpeed = Math.hypot(xTransSpeed.getAsDouble(), yTransSpeed.getAsDouble());
        double angle = Math.atan2(yTransSpeed.getAsDouble(), xTransSpeed.getAsDouble());
        double speed = xTransSpeed.getAsDouble();
        WaypointConstraints waypointConstraints = getCurrentWaypoint().getTranslationConstraints();

        if(Math.abs(combinedSpeed) > waypointConstraints.getMaxSpeed()){
            if(combinedSpeed > waypointConstraints.getMaxSpeed()){
                speed = Math.cos(angle) * waypointConstraints.getMaxSpeed();
            }else {
                speed = Math.cos(angle) * -waypointConstraints.getMaxSpeed();
            }
        }
        
        if(Math.abs(combinedSpeed) < waypointConstraints.getMinSpeed()){
            if(combinedSpeed > 0){
                speed = Math.cos(angle) * waypointConstraints.getMinSpeed();
            }else{
                speed = Math.cos(angle) * -waypointConstraints.getMinSpeed();
            }
        }

        return speed;
    }

    /**
     * Returns in meters per second.
     */
    public double getTranslationYSpeed(){
        double combinedSpeed = Math.hypot(xTransSpeed.getAsDouble(), yTransSpeed.getAsDouble());
        double angle = Math.atan2(yTransSpeed.getAsDouble(), xTransSpeed.getAsDouble());
        double speed = yTransSpeed.getAsDouble();
        WaypointConstraints waypointConstraints = getCurrentWaypoint().getTranslationConstraints();
        
        if(Math.abs(combinedSpeed) > waypointConstraints.getMaxSpeed()){
            if(combinedSpeed > waypointConstraints.getMaxSpeed()){
                speed = Math.sin(angle) * waypointConstraints.getMaxSpeed();
            }else {
                speed = Math.sin(angle) * -waypointConstraints.getMaxSpeed();
            }
        }
        
        if(Math.abs(combinedSpeed) < waypointConstraints.getMinSpeed()){
            if(combinedSpeed > 0){
                speed = Math.sin(angle) * waypointConstraints.getMinSpeed();
            }else{
                speed = Math.sin(angle) * -waypointConstraints.getMinSpeed();
            }
        }

        return speed;
    }

    /**
     * Sets the suppliers for speed on the robot to be equal to local variables.
     * @param xSpeed -X axis speed in meters per second.
     * @param ySpeed -Y axis speed in meters per second.
     * @param rotSpeed -Rotation speed in digrees per second.
     */
    public void setSpeedSuppliers(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed){
        xTransSpeed = xSpeed;
        yTransSpeed = ySpeed;
        this.rotSpeed = rotSpeed;
    }

    /**
     * Sets the path robot position equal to an outside pose supplier.
     */
    public void setPoseSupplier(Supplier<Pose2d> pose){
        robotPose = pose;
    }
    
    /**
     * Returns if the path is finished or not
     */
    public boolean isPathFinished(){
        return isPathFinished;
    }

    /**
     * Resets key variables
     */
    public void initialize(){
        currentWaypointIndex = 0;
        isPathFinished = false;
    }

    /**
     * Flips the waypoint positions and rotations of all waypoints
     */
    public void flipPath(boolean isRedAlliance, boolean isMirrored){
        
        //X and y from origins.
        double length = mapLengthX;
        double width = mapLengthY;
        if(isflipped != isRedAlliance){
            for(int i = 0; i < waypoints.length; i++) {
                //Gets waypoint position
                double waypointX = waypoints[i].getPose().getX();
                double waypointY = waypoints[i].getPose().getY();
                double waypointRot = waypoints[i].getPose().getRotation().getDegrees();

                double newWayX;
                double newWayY;
                double newWayRot;
                
                //flips
                newWayX = -waypointX + length;
                newWayY = waypointY;
            
                //Rotation flipping
                
                newWayRot = (waypointRot + 180) * -1;
                newWayRot = newWayRot % 180; //Sets angle to within 360.

                if(!isMirrored){
                    newWayY = -waypointY + width;
                    newWayRot = (waypointRot + 180) % 180;

                    if(!isRedAlliance){
                        newWayRot -= 180;
                    }
                }

                Waypoint newWaypoint = new Waypoint(new Pose2d(newWayX, newWayY, Rotation2d.fromDegrees(newWayRot)), 
                    waypoints[i].getTranslationConstraints(), waypoints[i].getRotationConstraints());

                waypoints[i] = newWaypoint;
                System.out.println("waypoint flipped");
                
            }
            isflipped = isRedAlliance;
        }
    }

    /**
     * Routs around any objects in the path
     */
    public void avoidFieldElements(boolean avoidFieldElements, FieldElement[] fieldElements, Path path, Pose2d suppliedPose){
        
    }

    public Waypoint[] getWaypoint() {
        if (nodeWaypoints != null) {
            System.out.println("nodeWaypoints: " + nodeWaypoints);
            return nodeWaypoints;
        } else {
            System.out.println("tempWaypoints: " + tempWaypoint);
            return tempWaypoint;
        }
    }
    /**
     * indexes waypoint to path to if there. 
     * If path is finished, sets path to finished and will not path to new waypoint.
     */
    public void periodic(){
        if(robotPose == null){
            System.out.println("RobotPose null");
            return;
        }
        if(waypoints == null){
            System.out.println("waypoints null");
            return;
        }
        if(waypoints[currentWaypointIndex].isAtWaypoint(robotPose.get()))
        {
            System.out.println("Next Waypoint");
            if(currentWaypointIndex == waypoints.length - 1){
                isPathFinished = true;
                System.out.println("Path Finished");
                return;
            }
            currentWaypointIndex++;
        }
    }
}
