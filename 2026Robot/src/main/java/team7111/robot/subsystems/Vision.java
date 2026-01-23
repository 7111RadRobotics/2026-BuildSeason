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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import team7111.robot.Constants;
import team7111.robot.utils.Camera;

public class Vision extends SubsystemBase{
    //we on longer have a limelight on the robot, however we may one day need to put it back on again. Therefore, I have left this code inside of the program, although it may make it less readable, it could be useful one day. Thank you for taking the time to read this wonderful message and I hope you have a great day :D
    //private PhotonCamera camera1 = new PhotonCamera("photonvision1");

    //Basic shooting settings
    /** Offset from horizontal of the camera angle */
    private final double cameraOffset = 0.0;
    /** Offset from horizontal that the shooter's minimum value is */
    private final double shooterAngleOffset = 37.0;
    /** Offset added to the camera to get to the shooter, in meters */
    private final double shooterXOffset = 0.0;
    /** Offset added to the camera to get to the shooter, in meters */
    private final double shooterZOffset = 0.0;
    /** Offset from the target to the aiming point, in meters. Increasing value makes the target higher than the apriltag*/
    private final double targetHeightOffset = 0.0;
    /** Offset from the target to the camera, in meters. Increasing value makes target farther away from apriltag  */
    private final double targetXOffset = 0.0;

    //Max and minimum angles
    private final double shooterMaxAngle = 67.0;
    private final double shooterMinAngle = 37.0;

    // Parabola constants
    /** Enable aiming using a parabola */
    private final boolean shootParabola = true;
    
    /** Minimum angle from horizontal (in degrees) the projectile can have at impact with the target */
    private final double minImpactAngle = 0;
    /** Maximum angle from horizontal (in degrees) the projectile can have at impact with the target */
    private final double maxImpactAngle = 90;
    /** On a scale of 0.0-1.0, how much rpm is lost when firing */
    private final double shootingLossPercent = 0.0;
    /** Diameter of the shooter wheel */
    private final double wheelDiameter = 4;

    /** Max height (in meters) ball may travel */
    private final double shootingMaxHeight = 10;
    /** Min height (in meters) ball may travel */
    private final double shootingMinHeight = 0.0;

    /** Sets the target height t */
    private final double targetHeight = 1;




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

    /**
     * Calculates distance to the target and uses a pre-determined formula to find the angle to hit the target
     * @implNote -MAKE SURE TO USE "shooterWheelSpeed" to set the firing speed before shooting;
     * @return value between 0 and 30 digrees
     */
    public double shooterAngle() {
        double calculatedAngle = shooterMinAngle;

        if(shootParabola) {
            calculatedAngle = shootParabola();
        } else {
            calculatedAngle = shootStraight();
        }
        

        return calculatedAngle;
    }

    /** Returns angle to shoot, at full power */
    private double shootStraight() {
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

        //Applies offsets
        calculatedAngle = shootingOffset(calculatedAngle);

        return calculatedAngle;
    }
    
    /**Shoots a parabolic tragectory to hit the target */
    double shootParabola() {
        
        //Position of the apriltag from the robot camera
        Transform3d aprilTagPos = cameraList[0].getCamToTarget();

        //Target position
        Pose3d targetPos = new Pose3d(aprilTagPos.getX() + targetXOffset, aprilTagPos.getY(), aprilTagPos.getZ() + targetHeightOffset, null);

        // calculates angle based on how close robot is to target
        double baseAngle;

        // calculates speed to make ball reach parabola's peak
        double baseSpeed;





        double calculatedAngle = 0.0;
        
        return calculatedAngle;
    }

    /** Adds offset from camera to shooter angle */
    private double shootingOffset(double input) {
        input = input + cameraOffset + shooterAngleOffset;
        
        input = 90 - input;

        if(input > shooterMaxAngle) {
            input = shooterMaxAngle;
        } else if(input < shooterMinAngle) {
            input = shooterMinAngle;
        }
        return input; 
    } 
}
