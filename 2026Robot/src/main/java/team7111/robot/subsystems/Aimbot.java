package team7111.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aimbot extends SubsystemBase{
    /** Given as backup for if camera detects no valid apriltag */
    private Supplier<Pose2d> robotPose;
    
    /** Possibly unused, determine with simon if we need to aim at specific point, or need apriltag detection*/
    private Vision vision;

    /** Position of target to aim at. */
    private Pose3d targetPose;
    private Translation2d parabTargetOffset;

    /** Only used with camera, distance to the center of the target from the apriltag */
    private final double camToTargetXOffset = 1.05918/2;
    /** Only used with camera, distance to the center of the target from the apriltag */
    private final double camToTargetHeightOffset = 2/3;

    /** shooter wheel diameter, in inches */
    private final double shooterDiameter = 4;
    /** Auto calculated based on shooter diameter, in inches */
    private final double wheelCircumference = shooterDiameter * Math.PI;

    /** Minimum shooter angle in degrees, from horizontal */
    private final double minShooterAngle = 0.0;
    /** Maximum shooter angle in degrees, from horizontal */
    private final double maxShooterAngle = 0.0;

    /** Offset from ground the ball leaves the shooter, in meters */
    private final double shooterHeightOffset = 0.75;
    /** Offset from the center of the robot to the shooter, in meters */
    private final double shooterXOffset = 0.25;
    
    /** Optimal rpm of the shooter wheel for max distance with continuous fire */
    private final double shooterOptimalSpeed = 10;

    /** How far from horizontal the camera is, in degrees */
    private double cameraAngleOffset = 0.0;
    /** How far from horizontal the camera is, in degrees*/
    private double shooterAngleOffset = 0.0;

    /** Enables/disables the math calculations (saves calculation time)
     *  <p> If disabled, sets angle to minimum shooter angle and speed to 0 */
    private boolean isEnabled = false;

    private boolean isUsingVision = false;

    /** Determines algorithm for aiming the shooter. <p>
     * Direct - Aims directly to the target, sets speed to max <p>
     * Parabolic - Aims to indirectly hit the target, arcing the ball <p>
     * Transport - Sets speed to 0, angle to the lowest possible <p>
     * Manual - Uses operator controls to aim and fire <p>
    */
    public enum shotType {
        Direct,
        Parabolic,
        Transport,
        Manual,
    }
    /** Current type of shot to calculate */
    private shotType currentShotType = shotType.Manual;

    /** Calculated angle set in periodic method, in degrees (includes shooter and camera offsets in calculation already) */
    private double calculatedAngle = 0.0;
    /** Calculated speed the wheel needs to spin at, in rotations per minute */
    private double calculatedSpeed = 0.0; 

    public Aimbot(Vision vision, Supplier<Pose2d> robotPose, Pose3d targetPose, Translation2d parabTargetOffset) {
        this.vision = vision;
        this.robotPose = robotPose;
        this.targetPose = targetPose;
        this.parabTargetOffset = parabTargetOffset;
    }

    /** Sets the angle offsets for the camera and the shooter, measured from horizontal */
    public void setOffsets(double cameraOffset, double shooterOffset) {
        this.cameraAngleOffset = cameraOffset;
        this.shooterAngleOffset = shooterOffset;
    }

    /** Returns calculated angle the shooter needs to fire at */
    public double getCalculatedAngle() {
        return this.calculatedAngle;
    }

    public double getCalculatedSpeed() {
        return this.calculatedSpeed;
    }

    /** Enables or disables the autoshooting calculations */
    public void setToggle(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }

    /** If set to true, vision will be used to find apriltag and target. If false, uses robot position */
    public void setVisionUsage(boolean isUsingVision) {
        this.isUsingVision = isUsingVision;
    }

    /** Calculates angle and speed for the shooter. If calculations are disabled, acts as a transport mode.*/
    public void periodic() {
        if(!isEnabled) {
            calculatedAngle = minShooterAngle;
            calculatedSpeed = 0;
            return;
        }
        
        // States to fire with
        switch (currentShotType) {
            case Direct:
                directShot();
                break;
            case Parabolic:
                parabolicShot();
                break;
            case Transport:
                transport();
                break;
            case Manual:
                manual();
                break;
        }

        useOffsets();
        useRestraints();
    }

    /** Aims directly at the target */
    private void directShot() {
        Transform3d distanceToTarget = getTransToTarget();
        
        if(distanceToTarget == null) {
            calculatedAngle = minShooterAngle;
            calculatedSpeed = 0;
            return;
        }

        double height = distanceToTarget.getZ();
        double distance = distanceToTarget.getX() + shooterXOffset;

        //Pathagorean theorum
        double directDistance = height * height + distance * distance;
        directDistance = Math.sqrt(directDistance);

        double calculatedAngle = Math.asin(height/directDistance);
        calculatedAngle = calculatedAngle * 180/Math.PI;

        calculatedSpeed = shooterOptimalSpeed;
    }

    /** Sets calculated angle and speed to arc a shot to the target. Uses a 3rd point that the ball will pass through to calculate.
     * <p> WARNING, Possibly heavy on processing */
    private void parabolicShot() {

        Transform3d CamToTarget = getTransToTarget();

        if(CamToTarget == null) {
            calculatedAngle = minShooterAngle;
            calculatedSpeed = 0;
            return;
        }

        double distanceToTarget = CamToTarget.getX() + shooterXOffset;
        double heightDifference = CamToTarget.getZ() - shooterHeightOffset;

        double distanceToHubEdge = distanceToTarget - 1;
        double targetHeightAboveHubEdge = 2.286; //90 inches above the floor

        double calculatedRatio = 
        ((distanceToTarget*distanceToTarget*heightDifference) - (distanceToHubEdge*distanceToHubEdge*targetHeightAboveHubEdge)) /
        ((distanceToHubEdge*distanceToTarget*(distanceToHubEdge-distanceToTarget)));

        calculatedAngle = Math.atan(calculatedRatio);

        double velocityCalculation = 
        (distanceToHubEdge * calculatedRatio - targetHeightAboveHubEdge) /
        ((1+calculatedRatio * calculatedRatio) * distanceToHubEdge * distanceToHubEdge);
        
        //Converts from meters per second to rotations per minute
        double velocityReq = Math.sqrt((9.81/(2*velocityCalculation)));
        velocityReq = velocityReq / (wheelCircumference / 12) * 3.281;
        calculatedSpeed = velocityReq / 60;
    }
    
    /** Sets angle to as close to horizontal as possible, and speed to 0 */
    private void transport() {
        calculatedAngle = minShooterAngle;
        calculatedSpeed = 0;
    }

    /** Sets to fire as flat of a line as possible. Operator controls do NOT determine raw angle, but distance they want to fire */
    private void manual() {

    }

    /** Offsets the angle to use the proper angle reference */
    private void useOffsets() {
        if(isUsingVision) {
            calculatedAngle = calculatedAngle + cameraAngleOffset;
        }
        calculatedAngle = calculatedAngle + shooterAngleOffset;
    }

    /** Ensures the angles are within the min and max physical angles on the shooter */
    private void useRestraints() {
        if(calculatedAngle < minShooterAngle) {
            calculatedAngle = minShooterAngle;
        } else if(calculatedAngle > maxShooterAngle) {
            calculatedAngle = maxShooterAngle;
        }
    }

    private Transform3d getTransToTarget() {
        
        if(isUsingVision) {
            Transform3d visionResults = vision.cameraList[0].getCamToTarget();
            if( visionResults == null) {
                return null;
            }
            Transform3d calculatedPos = new Transform3d(visionResults.getX() + camToTargetXOffset, visionResults.getY(), visionResults.getZ() + camToTargetHeightOffset, null);
            
            return calculatedPos;
        } else {
            Transform3d calculatedPos = new Transform3d(targetPose.getX() - robotPose.get().getX(),
            targetPose.getY() - robotPose.get().getY(),
            targetPose.getZ(), null);
            
            Transform3d returnedTrans = new Transform3d(Math.sqrt(Math.pow(calculatedPos.getX(), 2) + Math.pow(calculatedPos.getY(), 2)), 
                0.0, targetPose.getZ() - shooterHeightOffset, null);

            return returnedTrans;
        }
    }
}