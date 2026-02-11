package team7111.lib.pathfinding;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import team7111.robot.subsystems.SwerveSubsystem;

public class Pathfinding {

    // initialize pathfinding
    public Pathfinding(SwerveSubsystem swerve) {
        // set the default poses for pathfinding logic
        this.swerve = swerve;
        startPose = swerve.getPose();
        robotPose = swerve.getPose();
        gridPosition = new Translation2d(Math.round(robotPose.getX() / gridSize), Math.round(robotPose.getY() / gridSize));
        startPoseX = startPose.getX();
        startPoseY = startPose.getY();
    }

    // initalize swerve
    SwerveSubsystem swerve;

    // create int variables
    int dx;
    int dy;
    int iterations = 0;
    int running;
    int[][] directions = {
        {1, 0},
        {0, 1},
        {-1, 0},
        {0, -1},
        {1, 1},
        {-1, -1},
        {1, -1},
        {-1, 1},
    };

    // create translation2d varibables
    Translation2d wayPos = null;
    Translation2d gridPosition;
    
    // create pose2d variable
    Pose2d robotPose;
    Pose2d startPose;
    Pose2d positionOne;
    Pose2d positionTwo;
    Pose2d positionThree;

    // create list arrays
    List<Pose2d> storedPosition = new ArrayList<>();
    List<Pose2d> avoidPoses = new ArrayList<>();
    List<Translation2d> visitedNodes = new ArrayList<>();
    List<Translation2d> interiorWaypoints;

    // create boolean variables
    boolean isSafe;
    boolean visited;

    // create double variables
    double gridSize = 0.1;
    double robotRadius = 0.3;
    double currentDistance;
    double startDistance;
    double pathWeight;
    double safety = robotRadius + 0.1;
    double minScore = -1;
    double minScoreX;
    double minScoreY;
    double startPoseX;
    double startPoseY;
    double d1;
    double d2;
    double dis;
    double radius;
    double area;

    // create setAvoidPose to return the differnt points of the fieldElements
    private void setAvoidPose(FieldElement fieldElements) {
            Pose2d[] poses;

            // check if fieldElement has corners or if it is a circle 
            if (fieldElements.returnCorners() == null) {
                poses = fieldElements.returnCirclePose();
                radius = fieldElements.returnCircleRadius();
                safety = safety + radius;
            } else {
                poses = fieldElements.returnCorners();
            }

            // bad
            if (poses != null) {
                Collections.addAll(avoidPoses, poses);
            }
    }

    // create avoidFieldElements where pathfinding logic is calculated
    public Waypoint[] avoidFieldElements(Waypoint waypoint, FieldElement fieldElements) {
        // clear arrays to prevent errors
        visitedNodes.clear();
        storedPosition.clear();
        avoidPoses.clear();

        // reset numbers
        running = 1;
        iterations = 0;
        dis = 0;
        
        // reset poses and translations
        robotPose = swerve.getPose();
        gridPosition = robotPose.getTranslation();
        wayPos = waypoint.getPose().getTranslation();
    
        // resets setAvoidPose
        setAvoidPose(fieldElements);
    
        // checks if it isn't to close to end waypoint and if it hasn't run too many times
        while (robotPose.getTranslation().getDistance(wayPos) > 0.02 && iterations < 400) {
            // reset minscore
            minScore = -1;
            
            // runs once for reach direction in directions
            for (int[] dir : directions) {
                // reset booleans
                visited = false;
                isSafe = true;

                // calculate the neighbor poitions
                Translation2d neighbor = new Translation2d(
                    gridPosition.getX() + (dir[0] * gridSize), 
                    gridPosition.getY() + (dir[1] * gridSize)
                );

                // prevent revisiting
                for (Translation2d visit : visitedNodes) {
                    if (neighbor.getDistance(visit) < 0.05) {
                        visited = true;
                        break;
                    }
                }

                // prevent running into objects hopefully
                for (Pose2d obstacle : avoidPoses) {
                    if (neighbor.getDistance(obstacle.getTranslation()) < safety) {
                        isSafe = false;
                        break;
                    }
                }

                // stop if already visited
                if (visited) continue; 
                
                // continue if it isn't going ot run into anything
                if (isSafe) {

                    // calculate weights for pathfinding decisions
                    d1 = gridPosition.getDistance(neighbor) + dis;
                    d2 = neighbor.getDistance(wayPos);
                    pathWeight = d1 + d2;
                    
                    // set minscore values and prevent reseting
                    if (minScore == -1 || pathWeight < minScore) {
                        minScore = pathWeight;
                        minScoreX = neighbor.getX(); 
                        minScoreY = neighbor.getY();
                    }
                }
            }
    
            if (minScore != -1) {

                // set position values for calculating next moves
                gridPosition = new Translation2d(minScoreX, minScoreY);
                robotPose = new Pose2d(gridPosition, robotPose.getRotation());
                
                // add position values to arrays for easy calculating
                storedPosition.add(robotPose);
                visitedNodes.add(gridPosition);

                // distances
                dis += gridPosition.getDistance(new Translation2d(minScoreX, minScoreY));
            } else {
                // end if done
                break;
            }
            // to prevent 400 waypoints from being created
            iterations++;
        }
    
        // if no objects set to end waypoint
        if (storedPosition.isEmpty()) {
            return new Waypoint[] { waypoint };
        }
    
        // waypoints excluding beginning
        interiorWaypoints = new ArrayList<>();

        // runs code for each waypoint minus ending
        for (int i = 0; i < storedPosition.size() - 1; i++) {

            // prevent null out of bounds error
            while (i + 2 < storedPosition.size()) {

                // get future waypoint positions
                positionOne = storedPosition.get(i);
                positionTwo = storedPosition.get(i + 1);
                positionThree = storedPosition.get(i + 2);

                // calculate area
                area = (positionTwo.getY() - positionOne.getY()) * (positionThree.getX() - positionTwo.getX()) - 
                (positionTwo.getX() - positionOne.getX()) * (positionThree.getY() - positionTwo.getY());

                // remove waypoints in straight lines
                if (Math.abs(area) < 1e-9) {
                    storedPosition.remove(i + 1);
                } else {
                    break;
                }
            }

            // add waypoints to interiorWayponts
            interiorWaypoints.add(storedPosition.get(i).getTranslation());
        }

        // create waypoint array for path
        Waypoint[] path = new Waypoint[storedPosition.size()];

        // add waypoints to path
        for (int i = 0; i < storedPosition.size(); i++) {
            path[i] = new Waypoint(storedPosition.get(i), new WaypointConstraints(10, 0, 0.25), new WaypointConstraints(360, 0, 10));
        }

        // return finished path
        return path;
    }
}