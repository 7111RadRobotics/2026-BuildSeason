package team7111.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import team7111.lib.pathfinding.*;
import team7111.robot.Constants.SwerveConstants;
import team7111.robot.utils.SwerveModule;
import team7111.robot.utils.gyro.NavXGyro;
import team7111.robot.utils.gyro.SimSwerveGyro;
import team7111.robot.utils.gyro.GenericGyro;

public class Swerve extends SubsystemBase {
    private final SwerveModule[] modules;

    private final SwerveDrivePoseEstimator swerveOdometry;
    private Field2d field = new Field2d();

    private final GenericGyro gyro;
    private SwerveModuleState[] states = new SwerveModuleState[]{};

    private StructArrayPublisher<SwerveModuleState> commandedStatePublisher = 
            NetworkTableInstance.getDefault().getStructArrayTopic("Commanded Swerve States", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> actualStatePublisher = 
            NetworkTableInstance.getDefault().getStructArrayTopic("Actual Swerve States", SwerveModuleState.struct).publish();
    private StructPublisher<Pose2d> robotPosePublisher = 
            NetworkTableInstance.getDefault().getStructTopic("Robot Pose", Pose2d.struct).publish();
    
    private PathMaster pathMaster = null;

    private SwerveState currentSwerveState = SwerveState.manual;

    private Path path = null;

    private boolean isDriveFieldRelative;

    private DoubleSupplier joystickYTranslation = () -> 0;
    private DoubleSupplier joystickXTranslation = () -> 0;
    private DoubleSupplier joystickYaw = () -> 0;

    private double snapAngleSetpoint = 45;
    private PIDController snapAnglePID;

    private PIDController gamepieceAnglePID;
    private double gamepieceYaw = 0;

    private final double controllerDeadzone = 0.0;

    public enum SwerveState{
        initializePath,
        runPath,
        manual,
        stationary,
        snapAngle,
        bumpAlign,
        followGamePiece
    };

    public Swerve() {
        modules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.drivebaseConfig.moduleTypes[0]),
            new SwerveModule(1, SwerveConstants.drivebaseConfig.moduleTypes[1]),
            new SwerveModule(2, SwerveConstants.drivebaseConfig.moduleTypes[2]),
            new SwerveModule(3, SwerveConstants.drivebaseConfig.moduleTypes[3]),
        };

        gyro = RobotBase.isReal()
            ? new NavXGyro()
            : new SimSwerveGyro(this::getStates, SwerveConstants.kinematics);
        gyro.invertYaw(true);
        zeroGyro();

        pathMaster = new PathMaster(this::getPose, () -> getYaw());
        // Senior frog PID
        /*pathMaster.setTranslationPID(3, 0.015215, 0.1012);
        pathMaster.setRotationPID(0.1425, 0.000, 0.00240225);*/

        pathMaster.setTranslationPID(10, 0.0, 0);
        pathMaster.setRotationPID(0.5, 0, 0);
        pathMaster.setInversions(false, false, true, false);

        
        swerveOdometry = new SwerveDrivePoseEstimator(SwerveConstants.kinematics, getYaw(), 
            getPositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        
        snapAnglePID = new PIDController(0.04, 0.0, 0.001);
        gamepieceAnglePID = new PIDController(0.01, 0, 0);
    }

    @Override 
    public void periodic() {

        SmartDashboard.putNumber("Snap angle", snapAngleSetpoint);

        SmartDashboard.putNumber("Rotation", getYaw().getDegrees());

        gyro.update();
        swerveOdometry.update(getYaw(), getPositions());
        commandedStatePublisher.set(states);

        for(SwerveModule mod : modules){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getEncoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Meters", mod.getPosition().distanceMeters);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Rotations", mod.getPosition().distanceMeters / SwerveConstants.wheelCircumference);
        }
        SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());

        field.setRobotPose(getPose());
        SmartDashboard.putData(field);
        robotPosePublisher.set(getPose());

        actualStatePublisher.set(getStates());

        manageSwerveState();

        SmartDashboard.putString("swerveState", currentSwerveState.name());

        if(DriverStation.getAlliance().isPresent()){
            pathMaster.useAllianceFlipping(DriverStation.getAlliance().get() == Alliance.Red, true);
        }
    }

    public void simulationPeriodic(){
        for (SwerveModule mod : modules) {
            mod.module.update();
        }
    }

    public void manageSwerveState(){
        switch(currentSwerveState){
            case initializePath:
                if(path == null){
                    setSwerveState(SwerveState.stationary);
                    break;
                }
                pathMaster.initializePath(path);
                setSwerveState(SwerveState.runPath);
                break;
                
            case runPath:
                if(path == null){
                    setSwerveState(SwerveState.initializePath);
                    break;
                }
                if(path.isPathFinished()){
                    setSwerveState(SwerveState.stationary);
                    
                    break;
                }
                pathMaster.periodic(path);
                ChassisSpeeds speeds = pathMaster.getPathSpeeds(path, false, true);
                setModuleStates(SwerveConstants.kinematics.toSwerveModuleStates(speeds)); 
                if(path.getWaypoints().length == 0){
                    path = null;
                }
                break;

            case manual:
                manual(joystickXTranslation.getAsDouble(), joystickYTranslation.getAsDouble(), joystickYaw.getAsDouble(), isDriveFieldRelative, false);
                break;
            case stationary:
                manual(0, 0, 0, false, false);
                break;
            case snapAngle:
                manual(joystickXTranslation.getAsDouble(), joystickYTranslation.getAsDouble(), snapAnglePID.calculate(getYaw().getDegrees(), snapAngleSetpoint), isDriveFieldRelative, false);
                break;
            case followGamePiece:
                manual(joystickXTranslation.getAsDouble(), joystickYTranslation.getAsDouble(), gamepieceAnglePID.calculate(gamepieceYaw, snapAngleSetpoint), false, false);
                break;
            case bumpAlign:
                double angle = joystickYaw.getAsDouble();
                Pose2d pose = getPose();

                if ((pose.getY() > 1.4 && pose.getY() < 3.4) 
                 || (pose.getY() > 4.5 && pose.getY() < 6.5)){
                    if ((pose.getX() > 3.6 && pose.getX() < 5.5) 
                     || (pose.getX() > 11 && pose.getX() < 12.98)){
                        angle = snapAnglePID.calculate(-getYaw().getDegrees(), 45);
                    }
                }

                manual(joystickXTranslation.getAsDouble(), joystickYTranslation.getAsDouble(), angle, isDriveFieldRelative, false);
                break;
            default:
                break;
        }
    }

    public void manual(double forwardBack, double leftRight, double rotation, boolean isFieldRelative, boolean isOpenLoop){
        // Adding deadzone.
        forwardBack = Math.abs(forwardBack) < controllerDeadzone ? 0 : forwardBack;
        leftRight = Math.abs(leftRight) < controllerDeadzone ? 0 : leftRight;
        rotation = Math.abs(rotation) < controllerDeadzone ? 0 : rotation;

        double hypot = Math.hypot(leftRight, forwardBack);
        if(Math.abs(hypot) > 1){
            hypot = 1;
        }
        hypot = Math.pow(hypot, 3);
        double theta = Math.atan2(forwardBack, leftRight);

        forwardBack = hypot * Math.sin(theta);
        leftRight = hypot * Math.cos(theta);
        // Converting to m/s
        forwardBack *= SwerveConstants.maxDriveVelocity;
        leftRight *= SwerveConstants.maxDriveVelocity;
        rotation *= SwerveConstants.maxAngularVelocity;

        // Get desired module states.
        ChassisSpeeds chassisSpeeds = isFieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw())
            : new ChassisSpeeds(forwardBack, leftRight, rotation);

        SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(states, isOpenLoop);
    }

    public void setSwerveState(SwerveState swerveState){
        currentSwerveState = swerveState;
    }

    public SwerveState getSwerveState(){
        return currentSwerveState;
    }
    
    public void setSnapAngle(double snapNumber) {
        snapAngleSetpoint = snapNumber;
    }

    public void setGamepieceYaw(double yaw){
        gamepieceYaw = yaw;
    }

    public void addVisionMeasurement(Pose2d pose){
        swerveOdometry.addVisionMeasurement(new Pose2d(pose.getX(), pose.getY(), getYaw()), Timer.getFPGATimestamp());
    }

    /** To be used by auto. Use the drive method during teleop. */
    public void setModuleStates(SwerveModuleState[] states) {
        setModuleStates(states, false);
    }

    private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        // Makes sure the module states don't exceed the max speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxDriveVelocity);
        this.states = states;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[modules[i].moduleNumber], isOpenLoop);
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            currentStates[i] = modules[i].getState();
        }
        return currentStates;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            currentStates[i] = modules[i].getPosition();
        }

        return currentStates;
    }

    public Rotation2d getYaw() {
        return gyro.getYaw(); //Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Pose2d getPose() {
        return swerveOdometry.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public void setJoysickInputs(DoubleSupplier joystickYTranslation, DoubleSupplier joystickXTranslation, DoubleSupplier joystickYaw){
        this.joystickXTranslation = joystickXTranslation;
        this.joystickYTranslation = joystickYTranslation;
        this.joystickYaw = joystickYaw;
    }

    public void setDriveFieldRelative(boolean isFieldRelative){
        isDriveFieldRelative = isFieldRelative;
    }

    public void zeroGyro() {
        gyro.setYaw(Rotation2d.kZero);
        
    }

    

    public void setPath(Path path){
        this.path = path;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        for (SwerveModule module : modules) {
            builder.addDoubleProperty(
            String.format("Drive Pos %d", module.moduleNumber),
            () -> module.getPosition().distanceMeters,
            null);

            
            builder.addDoubleProperty(
            String.format("Angle %d", module.moduleNumber),
            () -> module.getAngle().getDegrees(),
            null);
        }
    }
}
