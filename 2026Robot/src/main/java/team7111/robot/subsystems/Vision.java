package team7111.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

    /** Constructor */
    public Vision(){

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
        }
    }
}
