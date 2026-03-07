package team7111.robot.subsystems;

import java.util.concurrent.TimeoutException;
import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aimbot extends SubsystemBase{
    /** Given as backup for if camera detects no valid apriltag */
    private Supplier<Pose2d> robotPose;
    private Supplier<Transform2d> robotVelocity;
    
    private final Pose3d blueHub = new Pose3d(4.635, 4.034536, Units.inchesToMeters(72), null); 
    private final Pose3d redHub = new Pose3d(11.946, 4.034536, Units.inchesToMeters(72), null); 
    
    /** Distance from the wall on each alliance to shoot at when passing */
    private final double edgeOffset = 1.0;
    /** Y offset of targets for each corner when passing, from the center of the field's Y pos */
    private final double yOffset = 4.034536 / 2.0;
    /** Blue alliance middle, red alliance middle */
    private final Pose3d[] corners = {new Pose3d(edgeOffset, 4.034536, 0, null), 
                                      new Pose3d(16.540988-edgeOffset, 4.034536, 0, null)};

    //CONTROLLER
    /** Controls the manual firing, and adds angle if the stick is moved */
    private XboxController operatorController = null;
    /** Multiplied against the angle stick (left). Called 50x per second, so max rate is this number x50 */
    private double angleSensitivity = 0.005;
    /** Multiplied against the speed stick (Right). Called 50x per second, so max rate is this number x50 */
    private double speedSensitivity = 0.000001;

    /** On scale of 0 - 1, if the controller is less than this, ignores the value */
    private double controllerDeadzone = 0.1;

    /** How far the stick can override the angle in non manual shots (degrees) */
    private double angleOverrideRange = 10;
    /** How far the operator can override the speed in non manual shots (rpm) */
    private double speedOverrideRange = 1000;


    /** For apriltag detection and targetting */
    private Vision vision;

    /** Position of target to aim at. */
    private Pose3d targetPose;

    /** Only used with camera, distance to the center of the target from the apriltag */
    private final double camToTargetXOffset = 1.05918/2;
    /** Only used with camera, distance to the center of the target from the apriltag */
    private final double camToTargetHeightOffset = 2/3;

    /** Offset between rio yaw and direction the robot shoots */
    private final double rioToShooterOffset = 0.0;

    /** shooter wheel diameter, in meters */
    private final double shooterDiameter = Units.inchesToMeters(4);
    /** Auto calculated based on shooter diameter, in inches */
    private final double wheelCircumference = shooterDiameter * Math.PI;

    //ANGLE CONSTRAINTS
    /** Minimum shooter angle in degrees, from horizontal */
    private final double minShooterAngle = 59.038;
    /** Maximum shooter angle in degrees, from horizontal */
    private final double maxShooterAngle = 83;
    
    private final double lowestShooterAngle = maxShooterAngle;
    //SPEED CONSTRAINTS
    /** Maximum rotations per minute allowable on the shooter (in RPM) */
    private final double maxShooterSpeed = 3000;
    /** Minimum rotations per minute allowable on the shooter (Overrided in off state, in RPM) */
    private final double minShooterSpeed = 0;

    //POSITION OFFSETS
    /** Offset from ground the ball leaves the shooter, in meters */
    private final double shooterHeightOffset = 0.75;
    /** Offset from the center of the robot to the shooter, in meters */
    private final double shooterXOffset = 0.25;
    
    /** Optimal rpm of the shooter wheel for max distance with continuous fire, in rotations per minute */
    private final double shooterOptimalSpeed = 1500;

    /** Extra multiplier to account for losses from drag, rpm loss from ball, ect */
    private final double RPMMult = 1.72;

    /** How far from horizontal the camera is, in degrees */
    private double cameraAngleOffset = 0.0;
    /** How far from horizontal the shooter is, in degrees */
    private double shooterAngleOffset = 0.0;

    /** Enables/disables the math calculations (saves calculation time)
     *  <p> If disabled, sets angle to minimum shooter angle and speed to 0 */
    private boolean isEnabled = true;

    /** Defines if we want to use vision */
    private boolean isUsingVision = false;

    /** Determines algorithm for aiming the shooter. <p>
     * Direct - Aims directly to the target, sets speed to max <p>
     * Parabolic - Aims to indirectly hit the target, arcing the ball <p>
     * Transport - Sets speed to 0, angle to the lowest possible <p>
     * Manual - Uses operator controls to aim and fire <p>
     * Apriltag - Targets directly to the most well seen apriltag, or continues current values if vision is disabled <p>
     * Preset - aims at the current preset shot type <p>
     * ShotTable - uses a shot table and interpolates where it should aim <p>
     * ShootOnTheMove - Uses leading to fire a ball rather than directly at the target. Also will shoot at corners if the robot is not in its alliance <p>
    */
    public enum shotType {
        Direct,
        Parabolic,
        Transport,
        Manual,
        Apriltag,
        Preset,
        ShotTable,
        ShootOnTheMove,
    }

    public enum presetShotType {
        Trench,
        RegHubShot,
        Pass,
        Default,
    }

    /** each double is the angle per foot */
    private double[] shotTableAngles = {
        83.0, //1ft
        78.8, //2ft
        76.0, //3ft
        73.0, //4ft
        72.0, //5ft
        70.8, //6ft
        69.7, //7ft
        68.8, //8ft
        67.9, //9ft
        67.2, //10ft
        66.6, //11ft
        66.1, //12ft
        65.7, //13ft
        65.3, //14ft
        64.9, //15ft
    };

    /** each double is the speed per foot */
    private double[] shotTableSpeeds = {
        582.0, //1ft
        551.0, //2ft
        557.5, //3ft
        573.1, //4ft
        592.0, //5ft
        612.2, //6ft
        632.8, //7ft
        653.4, //8ft
        673.8, //9ft
        693.9, //10ft
        713.7, //11ft
        733.1, //12ft
        752.1, //13ft
        770.8, //14ft
        789.1, //15ft
        };

    /** max distance for the shot table in feet */
    private final int maxDist = 15; 
    /** min distance for the shot table in feet */
    private final int minDist = 1;

    // On the move code
    /** Time per iteration in moving shot code, in seconds */
    private final double timeStep = 0.05;
    /** Max time in seconds the robot will calculate, in seconds */
    private final double maxFltTime = 6;
    /** Starting calculation step for the shooter, in seconds */
    private final double startingTimeStep = 2.0;
    /** Minimum angle the ball is allowed to fall into the target while using on the move shooting, in degrees from horizontal (positive is downwards) */
    private final double minImpactAngle = 65;
    /** If no valid shooting solution is found, sets this to false. */
    private boolean possibleToFire = false;

    /** Used to see if the target has changed at all, set each time it goes through the loop */
    private Pose3d prevTarget = null;
    /** If the firing stays on on the move shooting, turns to true. */
    private boolean uninterruptedFiring = false;
    /** If has calculated the time to fire a ball already, and still using the same target and firing method (most likely havent moved much) */
    private double timeOffset = 0;

    /** Current type of shot to calculate */
    private shotType currentShotType = shotType.Parabolic;

    /** Current preset to fire at if shot type is set to preset*/
    private presetShotType presetShot = presetShotType.Default;

    /** Calculated angle set in periodic method, in degrees (includes shooter and camera offsets in calculation already) */
    private double calculatedAngle = 0.0;
    /** Calculated speed the wheel needs to spin at, in rotations per minute */
    private double calculatedSpeed = 0.0; 
    /** The direction in degrees to the target */
    private double degreeToTarget = 0.0;

    public Aimbot(Vision vision, Supplier<Pose2d> robotPose, Supplier<Transform2d> robotVelocity) {
        this.vision = vision;
        this.robotPose = robotPose;
        this.robotVelocity = robotVelocity;
        
        //Defaults to shooting at the blue hub
        this.targetPose = blueHub;
    }

    /** Sets suppliers if not able to be given when aimbot class is initilized */
    public void giveResources(XboxController operatorController, BooleanSupplier isBlueAlliance) {
        this.operatorController = operatorController;
        if (isBlueAlliance != null) {
            if(isBlueAlliance.getAsBoolean()) {
            this.targetPose = blueHub;
        } else {
            this.targetPose = redHub;
        }

        }
        
    }

    /** Sets the angle offsets for the camera and the shooter, measured from horizontal */
    public void setOffsets(double cameraOffset, double shooterOffset) {
        this.cameraAngleOffset = cameraOffset;
        this.shooterAngleOffset = shooterOffset;
    }

    /** Returns if a shooting solution has been found. Only used with on the move shooting. */
    public boolean shotPossibleOnTheMove() {
        return possibleToFire;
    }

    /** Returns calculated angle the shooter needs to fire at */
    public double getCalculatedAngle() {
        return this.calculatedAngle;
    }

    public double getCalculatedSpeed() {
        return this.calculatedSpeed;
    }

    public double getCalculatedDirection() {

        // Checks and ensures between -180 and 180
        double returnAngle = degreeToTarget + rioToShooterOffset;
        if(returnAngle < -180) {
            returnAngle += 360;
        } else if(returnAngle > 180) {
            returnAngle -= 360;
        }

        return returnAngle;
    }

    public boolean getToggle(){
        return isEnabled;
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

    public void setPreset(presetShotType shotType) {
        presetShot = shotType;
    }

    /** Allows custom targeting to a target position. Does not affect anything if shooting with vision. */
    public void setCustomTarget(Pose3d customTarget) {
        targetPose = customTarget;
    }
    
    /** Resets the target to default (the current alliance hub for most shooting modes, unless otherwise specified) */
    public void resetTarget() {
        if (DriverStation.getAlliance().isPresent()) {
                
                if(DriverStation.getAlliance().get() == Alliance.Blue) {
                targetPose = blueHub;
            } else {
                targetPose = redHub;
            }
        } else {
            targetPose = blueHub;
        }
        
    }

    /** Calculates angle and speed for the shooter. If calculations are disabled, acts as a transport mode.*/
    public void periodic() {
        SmartDashboard.putBoolean("Is enabled", isEnabled);
        SmartDashboard.putBoolean("Is vision enabled", isUsingVision);
        SmartDashboard.putNumber("angle to target", degreeToTarget);

        if(!isEnabled) {
            calculatedAngle = lowestShooterAngle;
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
                SmartDashboard.putBoolean("ShootApril", false);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case Parabolic:
                parabolicShot();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", true);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", false);
                SmartDashboard.putBoolean("ShootApril", false);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case Transport:
                transport();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", true);
                SmartDashboard.putBoolean("Manual", false);
                SmartDashboard.putBoolean("ShootApril", false);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case Manual:
                manual();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", true);
                SmartDashboard.putBoolean("ShootApril", false);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case Apriltag:
                apriltag();
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", false);
                SmartDashboard.putBoolean("ShootApril", true);
                SmartDashboard.putBoolean("On the move", false);
                break;
            case ShootOnTheMove:
                shootOnTheMove();
                SmartDashboard.putBoolean("On the move", true);
                SmartDashboard.putBoolean("ShootDirect", false);
                SmartDashboard.putBoolean("ShootPara", false);
                SmartDashboard.putBoolean("Transport", false);
                SmartDashboard.putBoolean("Manual", false);
                SmartDashboard.putBoolean("ShootApril", false);
                break;
            case Preset:
                presetShot();
                break;
            case ShotTable:
                shotTable();
                break;
        }

        if(currentShotType != shotType.ShootOnTheMove) {
            uninterruptedFiring = false;
        }
        prevTarget = targetPose;

        useOffsets();
        useRestraints();
    }

    /** Aims using presets */
    private void presetShot() {

        switch (presetShot) {
            case Trench:
                calculatedSpeed = 2000;
                calculatedAngle = 60;
                break;
            case RegHubShot:
                calculatedAngle = 75;
                calculatedSpeed = 2000;
                break;
            case Pass:
                calculatedAngle = maxShooterAngle;
                calculatedSpeed = shooterOptimalSpeed;
                break;
            case Default:
                calculatedSpeed = 0;
                calculatedAngle = 0;
                break;
        }
    }

    /** Interpolates closest based on a shot table and current distance to target */
    private void shotTable() {
            Transform3d distanceToTarget = getTransToTarget();

            double distance = distanceToTarget.getX() + shooterXOffset;
            distance = Units.metersToFeet(distance);
            if(distance > maxDist || distance < minDist) {
                return;
            }

            double angleDifference = shotTableAngles[(int) Math.ceil(distance) -1] - shotTableAngles[(int) distance -1];
            double speedDifference = shotTableSpeeds[(int) Math.ceil(distance) -1] - shotTableSpeeds[(int) distance -1];

            double interpMult = distance % 1;

            angleDifference = angleDifference * interpMult;
            speedDifference = speedDifference * interpMult;

            calculatedAngle = shotTableAngles[(int) distance] + angleDifference;
            calculatedSpeed = shotTableSpeeds[(int) distance] + speedDifference;
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
        double targetHeightAboveHubEdge = Units.inchesToMeters(80); //90 inches above the floor, settable to anything desired for angle adjustment

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

        calculatedSpeed = velocityReq;
    }

    /** Most heavy on processing, MONITER SPEED OF PROCESSOR */
    public void shootOnTheMove() {

        /** If continuously firing, this is the time discrepancy it will give as padding to the previous time calculation.
         *  paddingTime/timeStep is a rough estemate of the time it will take per shot to calculate. Lower this value if too processor heavy
         */
        final double paddingTime = 0.25;

        possibleToFire = false;

        //Difference in poses to the target
        Transform3d difference = getTransToTarget();
    
        //Field relative, robotPose.get().getRotation() gets robot rotation.
        //X is forward/backward along the field (towards other alliance)
        //Y is left/right
        Transform2d robotVel = robotVelocity.get();

        double theta = Math.toRadians(degreeToTarget);

        double robotXVelocity =
            robotVel.getX() * Math.cos(theta) +
            robotVel.getY() * Math.sin(theta);

        double robotYVelocity =
            -robotVel.getX() * Math.sin(theta) +
            robotVel.getY() * Math.cos(theta);
        
        //Allows to skip
        if(!(uninterruptedFiring && prevTarget == targetPose)) {
            timeOffset = 0;
        }

        boolean wasWithinConstraints = false;
        //T is in a scale of timeStep
        for(double t = startingTimeStep + timeOffset; t < maxFltTime; t += timeStep) {

            //X is forward/back, Y is left/right, Z is height
            Transform3d outputVel = new Transform3d((difference.getX()/t) - robotXVelocity, 
                                                    (difference.getY()/t) - robotYVelocity, 
                                                    (difference.getZ()/t) - ((0.5)*-9.81*t),
                                                     null);

            double targetingAngleOffset = Math.atan2(outputVel.getY(), outputVel.getX());
            
            double shootingAngle = Math.atan2(outputVel.getZ(),
                                   Math.hypot(outputVel.getX(), outputVel.getY()));
            shootingAngle = Units.radiansToDegrees(shootingAngle);
            double shootingSpeed = Math.hypot(Math.hypot(outputVel.getZ(), outputVel.getY()), outputVel.getX());

            // m/s to rpm
            shootingSpeed = (shootingSpeed / wheelCircumference) * 60;

            //Constraints
            if(shootingSpeed <= maxShooterSpeed && shootingAngle >= minShooterAngle) {
                if(shootingAngle <= maxShooterAngle) {
                    double impactXVel = outputVel.getX();
                    double impactZVel = outputVel.getZ() - 9.81 * t;

                    double impactAngle = Units.radiansToDegrees(Math.atan2(-impactZVel, impactXVel));

                    wasWithinConstraints = true;

                    if(impactAngle > minImpactAngle) {
                        calculatedAngle = shootingAngle;
                        calculatedSpeed = shootingSpeed;
                        degreeToTarget += Units.radiansToDegrees(targetingAngleOffset);
                        timeOffset = t - startingTimeStep - paddingTime; //Optimization for firing continuously
                        uninterruptedFiring = true;
                        possibleToFire = true;
                        return;
                    }
                }
            } else if(wasWithinConstraints) {
                uninterruptedFiring = false;
                possibleToFire = false;
                break; //Quits the loop if we were in the constraints before, but went outside of one of them.
            }
            //If all 3 are true at once, will set calculated speed and angles.
            //If one boolean was true, but turns false (went out of bounds for angle or speed), then breaks early because no valid firing angle
        }
    }

    /** Sets angle to as close to horizontal as possible, and speed to 0 */
    private void transport() {
        calculatedAngle = lowestShooterAngle;
        calculatedSpeed = 0;
    }

    /** Sets to fire as flat of a line as possible. Operator controls do NOT determine raw angle, but distance they want to fire */
    private void manual() {
        //Deadzone application
        if(Math.abs(operatorController.getLeftY()) > controllerDeadzone) {
            calculatedAngle = calculatedAngle + operatorController.getLeftY() * angleSensitivity;
        }
        if(Math.abs(operatorController.getRightY()) > controllerDeadzone) {
            calculatedSpeed = operatorController.getRightY() * maxShooterSpeed;
        } else {
            calculatedSpeed = 0;
        }
    }

    /** Offsets the angle to use the proper angle reference */
    private void useOffsets() {
        if(isUsingVision) {
            calculatedAngle = calculatedAngle + cameraAngleOffset;
        }
        calculatedAngle = calculatedAngle + shooterAngleOffset;
        
        calculatedSpeed = calculatedSpeed *RPMMult;
        if(currentShotType != shotType.Manual)
        {
            //Deadzone application
            if(Math.abs(operatorController.getLeftY()) > controllerDeadzone) {
                calculatedAngle = calculatedAngle + operatorController.getLeftY() * angleOverrideRange / 2;
            }
            if(Math.abs(operatorController.getRightY()) > controllerDeadzone) {
                calculatedSpeed = calculatedSpeed + operatorController.getRightY() * speedOverrideRange / 2;
            }
        }
    }

    //Shoots towards apriltag, or last variables if apriltags are null/disabled
    void apriltag() {
        if(!isUsingVision) {
            return;
        }

        if(getTransToTarget() == null) {
            return;
        }

        directShot();
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
            Transform3d calculatedPos = new Transform3d(
                visionResults.getX() + camToTargetXOffset, //Distance to the target from center of robot
                visionResults.getY(), //Left/Right offset of the robot to the target
                visionResults.getZ() + camToTargetHeightOffset, //Height of the target (from 0)
                null);


            //FOR ROTATION CALUCLATION
            //X difference and y difference
            double xdif = targetPose.getX() - robotPose.get().getX();
            double ydif = targetPose.getY() - robotPose.get().getY();

            double rotation = 0;
            
            rotation = Math.toDegrees(Math.atan2(ydif, xdif));

            degreeToTarget = rotation;

            rotation = 90 - rotation;

            //ROTATION CALCULATION

            double distance = Math.sqrt(Math.pow(calculatedPos.getX(), 2) + Math.pow(calculatedPos.getY(), 2));
            Transform3d returnedTrans = new Transform3d(distance,
                0.0,
                calculatedPos.getZ(),
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
        
        rotation = Math.toDegrees(Math.atan2(ydif, xdif));

        degreeToTarget = rotation;

        rotation = 90 - rotation;

        double distance = Math.sqrt(Math.pow(calculatedPos.getX(), 2) + Math.pow(calculatedPos.getY(), 2));
        Transform3d returnedTrans = new Transform3d(
            distance, 
            0.0, 
            targetPose.getZ() - shooterHeightOffset, 
            new Rotation3d(new Rotation2d(rotation)));
        degreeToTarget = rotation;

        return returnedTrans;
    }
}
//floccinaucinihilipilification
//hehe >:3