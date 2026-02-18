package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.lang.annotation.Target;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import team7111.robot.utils.Camera;

public class Vision extends SubsystemBase{
    //we on longer have a limelight on the robot, however we may one day need to put it back on again. Therefore, I have left this code inside of the program, although it may make it less readable, it could be useful one day. Thank you for taking the time to read this wonderful message and I hope you have a great day :D
    //private PhotonCamera camera1 = new PhotonCamera("photonvision1");

    /**
     * Length is the number of cameras.
     * Each index is the position of the camera.
     * Add new cameras by extending the array
     */
    private final Transform3d cameraPositionsToCenter[] = {
        new Transform3d(Inches.of(14).in(Meters), Inches.of(8).in(Meters), 0, new Rotation3d(0, Degrees.of(15).in(Radians), 0)),
    };

    //private final AHRS gyro;
    public Pose2d robotPose = new Pose2d();
    public Pose3d estPose3d = new Pose3d();

    private List<PhotonTrackedTarget> targets = new ArrayList<>();

    // TODO: change variable names on actual robot
    /*public final Camera limelight = new Camera(
        "photonvision", 
        Constants.vision.cameraToRobotCenter1, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );*/
    public final Camera orangepi1;
    public final Camera orangepi2;

    public Camera[] cameraList;

    private double allowedPoseAmbiguity = 1;

    /** Constructor */
    public Vision(){
        targets.add(new PhotonTrackedTarget());

        orangepi1 = new Camera(
            "OV9281_3", 
            cameraPositionsToCenter[0], 
            new EstimatedRobotPose(estPose3d, 0.0, targets, PoseStrategy.AVERAGE_BEST_TARGETS), 
            this
        );

        orangepi2 = new Camera(
            "OV9281_2", 
            cameraPositionsToCenter[0], 
            new EstimatedRobotPose(estPose3d, 0.0, targets, PoseStrategy.AVERAGE_BEST_TARGETS), 
            this
        );


        cameraList = new Camera[] {
            orangepi2,
        };
    }

    public void periodic(){

        Optional<EstimatedRobotPose> estPose;

        for(Camera camera : cameraList){
            estPose = camera.getEstimatedGlobalPose(robotPose);
            robotPose = camera.estRobotPose.estimatedPose.toPose2d();
            if(estPose.isPresent()){
                if(estPose.get() != null)
                    camera.estRobotPose = estPose.get();
            }

            camera.periodic();
        }
    }

    /**
     * returns the position of the robot averaged between all cameras
     */
    public Pose3d getRobotPose() {
        
        boolean hasPose = false;
        double averageX = 0;
        double averageY = 0;
        double averageZ = 0;
        double averageRot = 0;
        int numOfCamerasSeen = 0;
        for(int i = 0; i < cameraList.length; i++) {
            if(cameraList[i].getLatestResult().hasTargets()) {
                hasPose = true;
                averageX += cameraList[i].estRobotPose.estimatedPose.getX();
                averageY += cameraList[i].estRobotPose.estimatedPose.getY();
                averageZ += cameraList[i].estRobotPose.estimatedPose.getZ();
                averageRot += cameraList[i].estRobotPose.estimatedPose.getRotation().getAngle();
                numOfCamerasSeen++;
            }
        }

        if(!hasPose) {
            return null;
        }

        averageX /= numOfCamerasSeen;
        averageY /= numOfCamerasSeen;
        averageZ /= numOfCamerasSeen;
        averageRot /= numOfCamerasSeen;

        Pose3d estPose = new Pose3d(averageX, averageY, averageZ, new Rotation3d(new Rotation2d(averageRot)));

        return hasPose 
            ? estPose
            : null;
    }

    /**
     * Returns null if the apriltag is not found.
     */
    public Pose3d getRobotPose(int apriltag) {
        int[] targetId = new int[cameraList.length];

        for(int i = 0; i < cameraList.length; i++) {
            for(int j = 0; j < cameraList[0].getLatestResult().targets.size(); j++) {
                if(cameraList[i].getLatestResult().getTargets().get(j).fiducialId == apriltag) {
                    targetId[i] = j;
                    break;
                } else {
                    targetId[i] = -1;
                }
            }
        }
        
        
        Transform3d transform = null;
        for(int i = 0; i < cameraList.length; i++) {
            if(targetId[i] != -1) {
                transform = cameraList[i].getLatestResult().getTargets().get(targetId[i]).getBestCameraToTarget();
                break;
            }
        }

        //If the target is not found, returns null
        if(transform == null) {
            return null;
        }

        Pose3d estPose = cameraList[0].getApriltagPos(apriltag).transformBy(transform);
        
        
        return estPose;
    }

    /** Gets the robot position from a specific camera*/
    public Pose3d getRobotPose(Camera camera, double ambiguityThreshold) {
        for(PhotonTrackedTarget target : camera.estRobotPose.targetsUsed){
            if(target.poseAmbiguity > ambiguityThreshold){
                return null;
            }
        }
        return camera.estRobotPose.estimatedPose;
    }
}
