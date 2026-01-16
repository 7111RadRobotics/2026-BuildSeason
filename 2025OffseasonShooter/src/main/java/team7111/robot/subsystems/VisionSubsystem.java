package team7111.robot.subsystems;

import java.util.Optional;
import java.lang.Math;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import team7111.robot.Constants;
import team7111.robot.utils.Camera;

public class VisionSubsystem extends SubsystemBase{
    //we on longer have a limelight on the robot, however we may one day need to put it back on again. Therefore, I have left this code inside of the program, although it may make it less readable, it could be useful one day. Thank you for taking the time to read this wonderful message and I hope you have a great day :D
    //private PhotonCamera camera1 = new PhotonCamera("photonvision1");
    
    /** Offset from horizontal of the camera angle */
    private final double cameraOffset = 0.0;
    /** Offset from horizontal that the shooter's minimum value is */
    private final double shooterAngleOffset = 37.0;
    /** Offset added to the camera to get to the shooter, in meters */
    private final double shooterXOffset = 0.0;
    /** Offset added to the camera to get to the shooter, in meters */
    private final double shooterZOffset = 0.0;
    /** Offset from the target to the aiming point, in meters*/
    private final double targetHeightOffset = 0.0;
    /** Offset from the target to the aiming, in meters  */
    private final double targetXOffset = 0.0;

    //Max and minimum angles
    private final double shooterMaxAngle = 67.0;
    private final double shooterMinAngle = 37.0;
    
    /**
     * Length is the number of cameras.
     * Each index is the position of the camera.
     * Add new cameras by extending the array
     */
    private final Transform3d cameraPositionsToCenter[] = {
        new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)),
    };

    //private final AHRS gyro;
    public Pose2d robotPose = new Pose2d();
    public Pose3d estPose3d = new Pose3d();

    // TODO: change variable names on actual robot
    /*public final Camera limelight = new Camera(
        "photonvision", 
        Constants.vision.cameraToRobotCenter1, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );*/
    public final Camera orangepi1 = new Camera(
        "OV9281_3", 
        cameraPositionsToCenter[0], 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );
    public final Camera orangepi2 = new Camera(
        "OV9281_2", 
        cameraPositionsToCenter[0], 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );

    public Camera[] cameraList = new Camera[] {
        orangepi1,

    };

    public VisionSubsystem(){

    }

    public void periodic(){

        Optional<EstimatedRobotPose> estPose;

        for(Camera camera : cameraList){
            estPose = camera.getEstimatedGlobalPose(robotPose);
            robotPose = camera.estRobotPose.estimatedPose.transformBy(camera.getCameraToRobot()).toPose2d();
            if(estPose.isPresent()){
                if(estPose.get() != null)
                    camera.estRobotPose = estPose.get();
            }

            camera.periodic();

            SmartDashboard.putNumber("Shooting Angle", shooterAngle());
        }
    }

    /**
     * Calculates distance to the target and uses a pre-determined formula to find the angle to hit the target
     * @return value between 0 and 30 digrees
     */
    public double shooterAngle() {
        //Z is height, X is distance, Y is X or y offset (irrelevant)

        Transform3d distanceToTarget = cameraList[0].getCamToTarget();

        if(distanceToTarget == null) {
            return 0;
        }

        double height = distanceToTarget.getZ() + shooterZOffset + targetHeightOffset;
        double distance = distanceToTarget.getX() + shooterXOffset + targetXOffset;

        double directDistance = height * height + distance * distance;
        directDistance = Math.sqrt(directDistance);

        double calculatedAngle = Math.asin(height/directDistance);
        calculatedAngle = calculatedAngle * 180/Math.PI;

        //Adds angle offsets
        calculatedAngle = calculatedAngle + cameraOffset + shooterAngleOffset;
        
        calculatedAngle = 90 - calculatedAngle;

        if(calculatedAngle > shooterMaxAngle) {
            calculatedAngle = shooterMaxAngle;
        } else if(calculatedAngle < shooterMinAngle) {
            calculatedAngle = shooterMinAngle;
        }


        return calculatedAngle;
    }
}
