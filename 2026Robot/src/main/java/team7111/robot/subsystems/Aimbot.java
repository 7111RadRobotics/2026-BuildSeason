package team7111.robot.subsystems;

import java.util.function.Supplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aimbot extends SubsystemBase{
    /** Given as backup for if camera detects no valid apriltag */
    private Supplier<Pose2d> robotPose;
    
    //CONTROLLER
    /** Controls the manual firing, and adds angle if the stick is moved */
    private XboxController operatorController = null;
    /** Multiplied against the angle stick (left). Called 50x per second, so max rate is this number x50 */
    private double angleSensitivity = 0.005;
    /** Multiplied against the speed stick (Right). Called 50x per second, so max rate is this number x50 */
    private double speedSensitivity = 0.001;
    /** How far the stick can override the angle in non manual shots (degrees) */
    private double angleOverrideRange = 10;
    /** How far the operator can override the speed in non manual shots (rpm) */
    private double speedOverrideRange = 500;


    /** For apriltag detection and targetting */
    private Vision vision;

    /** Position of target to aim at. */
    private Pose3d targetPose;

    /** Only used with camera, distance to the center of the target from the apriltag */
    private final double camToTargetXOffset = 1.05918/2;
    /** Only used with camera, distance to the center of the target from the apriltag */
    private final double camToTargetHeightOffset = 2/3;

    /** shooter wheel diameter, in meters */
    private final double shooterDiameter = Units.inchesToMeters(4);
    /** Auto calculated based on shooter diameter, in inches */
    private final double wheelCircumference = shooterDiameter * Math.PI;

    //ANGLE CONSTRAINTS
    /** Minimum shooter angle in degrees, from horizontal */
    private final double minShooterAngle = 37.0;
    /** Maximum shooter angle in degrees, from horizontal */
    private final double maxShooterAngle = 67.0;
    
    //SPEED CONSTRAINTS
    /** Maximum rotations per minute allowable on the shooter (in RPM) */
    private final double maxShooterSpeed = 2000;
    /** Minimum rotations per minute allowable on the shooter (Overrided in off state, in RPM) */
    private final double minShooterSpeed = 0;

    //POSITION OFFSETS
    /** Offset from ground the ball leaves the shooter, in meters */
    private final double shooterHeightOffset = 0.75;
    /** Offset from the center of the robot to the shooter, in meters */
    private final double shooterXOffset = 0.25;
    
    /** Optimal rpm of the shooter wheel for max distance with continuous fire, in rotations per minute */
    private final double shooterOptimalSpeed = 1500;

    /** How far from horizontal the camera is, in degrees */
    private double cameraAngleOffset = 0.0;
    /** How far from horizontal the shooter is, in degrees */
    private double shooterAngleOffset = 0.0;

    /** Enables/disables the math calculations (saves calculation time)
     *  <p> If disabled, sets angle to minimum shooter angle and speed to 0 */
    private boolean isEnabled = false;

    /** Defines if we want to use vision */
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
    private shotType currentShotType = shotType.Parabolic;

    /** Calculated angle set in periodic method, in degrees (includes shooter and camera offsets in calculation already) */
    private double calculatedAngle = 0.0;
    /** Calculated speed the wheel needs to spin at, in rotations per minute */
    private double calculatedSpeed = 0.0; 
    /** The direction in degrees to the target */
    private double degreeToTarget = 0.0;

    public Aimbot(Vision vision, Supplier<Pose2d> robotPose, Pose3d targetPose, XboxController operatorController) {
        this.vision = vision;
        this.robotPose = robotPose;
        this.targetPose = targetPose;
        this.operatorController = operatorController;
    }

    /** Sets suppliers if not able to be given when aimbot class is initilized */
    public void giveResources(XboxController operatorController) {
        this.operatorController = operatorController;
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

    public double getCalculatedDirection() {
        return this.degreeToTarget;
    }

    /** Enables or disables the autoshooting calculations */
    public void setToggle(boolean isEnabled) {
        this.isEnabled = isEnabled;
    }

    /** Toggles the enable for calculations */
    public void toggle() {
        this.isEnabled = !isEnabled;
    }

    /** If set to true, vision will be used to find apriltag and target. If false, uses robot position */
    public void setVisionUsage(boolean isUsingVision) {
        this.isUsingVision = isUsingVision;
    }

    /** Toggles the vision usage */
    public void toggleVision() {
        isUsingVision = !isUsingVision;
    }

    /** Sets the current shot type to calculate */
    public void setShotType(shotType shotType) {
        currentShotType = shotType;
    }

    /** Calculates angle and speed for the shooter. If calculations are disabled, acts as a transport mode.*/
    public void periodic() {
        SmartDashboard.putBoolean("Is enabled", isEnabled);
        SmartDashboard.putBoolean("Is vision enabled", isUsingVision);
        SmartDashboard.putNumber("angle to target", degreeToTarget);

        if(!isEnabled) {
            calculatedAngle = minShooterAngle;
            calculatedSpeed = 0;
            return;
        }
        
        // States to fire with
        switch (currentShotType) {
            case Direct:
                directShot();
                SmartDashboard.putBoolean("ShootDirect", true);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", false);
                break;
            case Parabolic:
                parabolicShot();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", true);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", false);
                break;
            case Transport:
                transport();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", true);
                SmartDashboard.putBoolean("Manual", false);
                break;
            case Manual:
                manual();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", true);
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

        calculatedAngle = Math.asin(height/directDistance);
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

        SmartDashboard.putNumber("Distance to target", distanceToTarget);
        //Subtracts a meter from the distance to get a 2nd target point
        double distanceToHubEdge = distanceToTarget - 0.5;
        double targetHeightAboveHubEdge = 2.286; //90 inches above the floor, settable to anything desired for angle adjustment

        double calculatedRatio = 
            ((distanceToTarget*distanceToTarget*heightDifference) - (distanceToHubEdge*distanceToHubEdge*targetHeightAboveHubEdge)) /
            ((distanceToHubEdge*distanceToTarget*(distanceToHubEdge-distanceToTarget)));

        if(calculatedRatio < 0) {
            calculatedAngle = minShooterAngle;
        } else {
            calculatedAngle = Math.atan(calculatedRatio) * 180/Math.PI;
        }
        
        SmartDashboard.putNumber("Nan?", calculatedRatio);

        double velocityCalculation = 
            (distanceToHubEdge * calculatedRatio - targetHeightAboveHubEdge) /
            ((1+calculatedRatio * calculatedRatio) * distanceToHubEdge * distanceToHubEdge);
        SmartDashboard.putNumber("Velocity calculation", velocityCalculation);
        
        //Converts from meters per second to rotations per minute
        double velocityReq = Math.sqrt((9.81/(2*velocityCalculation)));
        SmartDashboard.putNumber("VelocityRequired", velocityReq);

        velocityReq = velocityReq / wheelCircumference;
        SmartDashboard.putNumber("VelocityToRotationsPerMinute", velocityReq);

        calculatedSpeed = velocityReq * 60;
    }
    
    /** Sets angle to as close to horizontal as possible, and speed to 0 */
    private void transport() {
        calculatedAngle = minShooterAngle;
        calculatedSpeed = 0;
    }

    /** Sets to fire as flat of a line as possible. Operator controls do NOT determine raw angle, but distance they want to fire */
    private void manual() {
        calculatedAngle = calculatedAngle + operatorController.getLeftY() * angleSensitivity;
        calculatedSpeed = calculatedSpeed + operatorController.getRightY() * speedSensitivity;
    }

    /** Offsets the angle to use the proper angle reference */
    private void useOffsets() {
        if(isUsingVision) {
            calculatedAngle = calculatedAngle + cameraAngleOffset;
        }
        calculatedAngle = calculatedAngle + shooterAngleOffset;

        calculatedAngle = calculatedAngle + operatorController.getLeftY() * angleOverrideRange / 2;
        calculatedSpeed = calculatedSpeed + operatorController.getRightY() * speedOverrideRange / 2;
        SmartDashboard.putNumber("calculated angle untampered", calculatedAngle);
    }

    /** Ensures the angles are within the min and max physical angles on the shooter */
    private void useRestraints() {
        SmartDashboard.putNumber("CalculatedAngle before clamped", calculatedAngle);
        SmartDashboard.putNumber("CalculatedSpeed before clamped", calculatedSpeed);

        if(calculatedAngle < minShooterAngle) {
            calculatedAngle = minShooterAngle;
        } else if(calculatedAngle > maxShooterAngle) {
            calculatedAngle = maxShooterAngle;
        }

        if(calculatedSpeed > maxShooterSpeed) {
            calculatedSpeed = maxShooterSpeed;
        } else if(calculatedSpeed < minShooterSpeed) {
            calculatedSpeed = minShooterSpeed;
        }
    }

    /** Gets a transform 3d object to the target, either using vision or absolute position to calculate <p>
     * X is distance to the target,
     * Z is height off ground,
     * Rotation is the direction you need to be pointing at the target, not an offset, in degrees.
     */
    private Transform3d getTransToTarget() {
        
        if(isUsingVision) {
            Transform3d visionResults = vision.cameraList[0].getCamToTarget();
            if( visionResults == null) {
                return null;
            }
            //Robot relative
            Transform3d calculatedPos = new Transform3d(visionResults.getX() + camToTargetXOffset, //Distance to the target from center of robot
                visionResults.getY(), //Left/Right offset of the robot to the target
                visionResults.getZ() + camToTargetHeightOffset, //Height of the target (from 0)
                null);

            //X difference and y difference
            double xdif = targetPose.getX() - robotPose.get().getX();
            double ydif = targetPose.getY() - robotPose.get().getY();

            double rotation = 0;
            if(xdif > 0) {
                if(ydif > 0) {
                    rotation = Math.atan(xdif/ydif) * 180/Math.PI;
                } else {
                    rotation = Math.atan(-ydif/xdif) * 180/Math.PI + 90;
                }
            } else {
                if(ydif > 0) {
                    rotation = Math.atan(ydif/-xdif) * 180/Math.PI + 270;
                } else {
                    rotation = Math.atan(-xdif/-ydif) * 180/Math.PI + 180;
                }
            }

            double distance = Math.sqrt(Math.pow(calculatedPos.getX(), 2) + Math.pow(calculatedPos.getY(), 2));
            Transform3d returnedTrans = new Transform3d(distance,
                0.0,
                visionResults.getZ(),
                new Rotation3d(new Rotation2d(rotation)));
            degreeToTarget = rotation;

            return returnedTrans;
        }
        //Field relative
        Transform3d calculatedPos = new Transform3d(targetPose.getX() - robotPose.get().getX(), //Left/Right distance to the target
            targetPose.getY() - robotPose.get().getY(), //Up/Down offset in 2d grid
            targetPose.getZ(), //Height of the target (from 0)
            null);
            
        //X difference and y difference
        double xdif = targetPose.getX() - robotPose.get().getX();
        double ydif = targetPose.getY() - robotPose.get().getY();

        double rotation = 0;
        if(xdif > 0) {
            if(ydif > 0) {
                rotation = Math.atan(xdif/ydif) * 180/Math.PI;
            } else {
                rotation = Math.atan(-ydif/xdif) * 180/Math.PI + 90;
            }
        } else {
            //ISSUE:
            //Vision uses local x and y difference
            //This is using field relative x and y difference
            //Same reason angle is hard to calculate
            if(ydif > 0) {
                rotation = Math.atan(ydif/-xdif) * 180/Math.PI + 270;
            } else {
                rotation = Math.atan(-xdif/-ydif) * 180/Math.PI + 180;
            }
        }

        double distance = Math.sqrt(Math.pow(calculatedPos.getX(), 2) + Math.pow(calculatedPos.getY(), 2));
        Transform3d returnedTrans = new Transform3d(
            distance, 
            0.0, 
            targetPose.getZ() - shooterHeightOffset, 
            new Rotation3d(new Rotation2d(rotation)));
        degreeToTarget = rotation;

        return returnedTrans;
    }

    public XboxController getOperatorController(){
        return operatorController;
    }
}
//floccinaucinihilipilification