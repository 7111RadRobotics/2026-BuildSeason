package team7111.lib.pathfinding;

import edu.wpi.first.math.geometry.Pose2d;

public class Waypoint {
    private Pose2d pose;
    private WaypointConstraints transConstraints;
    private WaypointConstraints rotConstraints;

    /**
     * Constructs a new {@code Waypoint} with the given pose and tolerances/speeds.
     * <p> - By ChatGPT-o4 2025
     *
     * @param pose The desired position and orientation of the waypoint (as a {@code Pose2d}).
     * @param translationTolerance The allowable positional error in meters for reaching this waypoint.
     * @param rotationTolerance The allowable angular error in degrees for reaching this waypoint.
     * @param maxTranslationSpeed The maximum speed in meters per second to approach this waypoint.
     * @param maxRotationSpeed The maximum rotational speed in degrees per second when turning toward this waypoint.
     */
    public Waypoint(Pose2d pose, WaypointConstraints translationTolerances, WaypointConstraints rotationTolerances){
        this.pose = pose;
        this.transConstraints = translationTolerances;
        this.rotConstraints = rotationTolerances;
    }
    
    /**
     * Checks if the robot is within the bounds of the waypoint.
     * @param robotPose - The current pose of the robot.
     */
    public boolean isAtWaypoint(Pose2d robotPose){

        //Checks translation tolerance
        if (robotPose.getX() < (pose.getX() + transConstraints.getTolerance())
         && robotPose.getX() > (pose.getX() - transConstraints.getTolerance()) ){
            if (robotPose.getY() < (pose.getY() + transConstraints.getTolerance())
             && robotPose.getY() > (pose.getY() - transConstraints.getTolerance()) ){
                //Checks rotation tolerance
                if(robotPose.getRotation().getDegrees() < pose.getRotation().getDegrees() + rotConstraints.getTolerance())
                {
                    if(robotPose.getRotation().getDegrees() > pose.getRotation().getDegrees() - rotConstraints.getTolerance())
                    {
                        return true;
                    }
                }
            }
        }

        
        return false;
    }
    public WaypointConstraints getRotationConstraints(){
        return rotConstraints;
    }
    public WaypointConstraints getTranslationConstraints(){
        return transConstraints;
    }

    public Pose2d getPose(){
        return pose;
    }


}
