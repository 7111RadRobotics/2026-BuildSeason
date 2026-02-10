package team7111.lib.pathfinding;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.OptionalInt;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.RobotBase;
import team7111.robot.subsystems.SwerveSubsystem;

public class Pathfinding {
    SwerveSubsystem swerve;
    Pose2d robotPose;
    Pose2d startPose;
    Translation2d gridPosition;
    double startPoseX;
    double startPoseY;
    public Pathfinding(SwerveSubsystem swerve) {
        this.swerve = swerve;
        startPose = swerve.getPose();
        robotPose = swerve.getPose();
        gridPosition = new Translation2d(Math.round(robotPose.getX() / gridSize), Math.round(robotPose.getY() / gridSize));
        startPoseX = startPose.getX();
        startPoseY = startPose.getY();
    }
    Translation2d wayPos = null;
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
    double gridSize = 0.1;
    double robotRadius = 0.3;
    double currentDistance;
    double startDistance;
    double pathWeight;
    double safety = robotRadius + 0.1;
    double minScore = -1;
    double minScoreX;
    double minScoreY;
    int dx;
    int dy;
    boolean isSafe;
    boolean visited;
    List<Pose2d> storedPosition = new ArrayList<>();
    List<Translation2d> visitedNodes = new ArrayList<>();

    List<Pose2d> avoidPoses = new ArrayList<>();

    int iterations = 0;

    double d1;
    double d2;
    double dis;
    Pose2d positionOne;
    Pose2d positionTwo;
    Pose2d positionThree;
    double radius;
    double area;
    int running;

    List<Translation2d> interiorWaypoints;

    private void setAvoidPose(FieldElement fieldElements) {
            Pose2d[] poses;
            if (fieldElements.returnCorners() == null) {
                poses = fieldElements.returnCirclePose();
                radius = fieldElements.returnCircleRadius();
                safety = safety + radius;
            } else {
                poses = fieldElements.returnCorners();
            }
            if (poses != null) {
                Collections.addAll(avoidPoses, poses);
            }
    }

    public Waypoint[] avoidFieldElements(Waypoint waypoint, FieldElement fieldElements) {
        visitedNodes.clear();
        storedPosition.clear();
        avoidPoses.clear();
        iterations = 0;
        dis = 0;
        running = 1;
        
        robotPose = swerve.getPose();
        gridPosition = robotPose.getTranslation();
        wayPos = waypoint.getPose().getTranslation();
    
        setAvoidPose(fieldElements);
    
        while (robotPose.getTranslation().getDistance(wayPos) > 0.02 && iterations < 400) {
            minScore = -1;
    
            for (int[] dir : directions) {
                Translation2d neighbor = new Translation2d(
                    gridPosition.getX() + (dir[0] * gridSize), 
                    gridPosition.getY() + (dir[1] * gridSize)
                );
    
                visited = false;
                for (Translation2d visit : visitedNodes) {
                    if (neighbor.getDistance(visit) < 0.05) {
                        visited = true;
                        break;
                    }
                }
                if (visited) continue; 
    
                isSafe = true;
                for (Pose2d obstacle : avoidPoses) {
                    if (neighbor.getDistance(obstacle.getTranslation()) < safety) {
                        isSafe = false;
                        break;
                    }
                }
    
                if (isSafe) {
                    d1 = gridPosition.getDistance(neighbor) + dis;
                    d2 = neighbor.getDistance(wayPos);
                    pathWeight = d1 + d2;
    
                    if (minScore == -1 || pathWeight < minScore) {
                        minScore = pathWeight;
                        minScoreX = neighbor.getX(); 
                        minScoreY = neighbor.getY();
                    }
                }
            }
    
            if (minScore != -1) {
                gridPosition = new Translation2d(minScoreX, minScoreY);
                robotPose = new Pose2d(gridPosition, robotPose.getRotation());
                
                storedPosition.add(robotPose);
                visitedNodes.add(gridPosition);
                dis += gridPosition.getDistance(new Translation2d(minScoreX, minScoreY));
            } else {
                break;
            }
            iterations++;
        }
    
        if (storedPosition.isEmpty()) {
            return new Waypoint[] { waypoint };
        }
    
        interiorWaypoints = new ArrayList<>();

        for (int i = 0; i < storedPosition.size() - 1; i++) {
            interiorWaypoints.add(storedPosition.get(i).getTranslation());

            while (i + 2 < storedPosition.size()) {
                positionOne = storedPosition.get(i);
                positionTwo = storedPosition.get(i + 1);
                positionThree = storedPosition.get(i + 2);

                area = (positionTwo.getY() - positionOne.getY()) * (positionThree.getX() - positionTwo.getX()) - 
                (positionTwo.getX() - positionOne.getX()) * (positionThree.getY() - positionTwo.getY());

                if (Math.abs(area) < 1e-9) {
                    storedPosition.remove(i + 1);
                } else {
                    break;
                }
            }
            interiorWaypoints.add(storedPosition.get(i).getTranslation());
        }
        Waypoint[] path = new Waypoint[storedPosition.size()];

        for (int i = 0; i < storedPosition.size(); i++) {
            path[i] = new Waypoint(storedPosition.get(i), new WaypointConstraints(10, 0, 0.25), new WaypointConstraints(360, 0, 10));
        }

        return path;
    }
}