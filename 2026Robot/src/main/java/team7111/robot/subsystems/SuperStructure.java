package team7111.robot.subsystems;

import java.util.List;

import javax.print.attribute.standard.RequestingUserName;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.subsystems.Aimbot.shotType;
import team7111.robot.subsystems.Aimbot.presetShotType;
import team7111.robot.subsystems.Autonomous.Paths;
import team7111.robot.subsystems.Hopper.HopperState;
import team7111.robot.subsystems.Intake.IntakeState;
import team7111.robot.subsystems.Shooter.ShooterState;
import team7111.robot.subsystems.Swerve.SwerveState;
import team7111.robot.utils.AutoAction;
import team7111.lib.pathfinding.*;

/**
 * This class handles the overall state of the robot.
 * It does this by defining various "SuperState" values and calling the methods assosiated with each one.
 * Each method uses logic to determine subsystem states, when to switch SuperStates, and what SuperState to go to next.
 * Logic used can be button inputs, subsystem states, or other subsystem conditions.
 */
public class SuperStructure extends SubsystemBase {
    /**
     * This enumeration contains the values or "states" which determine the subsystems.
     * The top 2 state names are temporary states for testing 
     */
    public enum SuperState {
        stowed,
        deployed,
        intake,
        pass,
        score,
        snowBlowerPass,
        snowBlowerScore,
        prepareHubShot,
        preparePass,
        autonomousEnter,
        autonomous,
        autonomousExit,
        manual,
    }

    //Subsystem Variables
    private final Autonomous auto;
    private final Swerve swerve;
    private final Vision vision;
    private final Aimbot targeting;
    private final Intake intake;
    private final Hopper hopper;
    private final Shooter shooter;

    // Buttons of controllers can be assigned to booleans which are checked in various super states. 
    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    /** This represents the current superstate of the robot */
    private SuperState superState = SuperState.stowed;
    private boolean hasAcheivedState = false;

    private boolean inAuto = false;
    private int autoIndex = 0;
    private List<AutoAction> autoActions;

    private boolean aimAtHub = false;
    private boolean alignToHub = false;
    private boolean moveThroughTrench = false;
    private boolean orientWithBump = false;
    private boolean useObjectDetection = false;
    private boolean defaultDrive = true;
    private boolean trenchShot = false;

    private boolean intaking = false;
    private boolean scoring = false;
    private boolean passing = false;
    private boolean stow = false;

    private shotType currentShot = shotType.Transport;
    private shotType scoringState = shotType.Direct;

    /**
     * The constructor will take each subsystem as an argument and save them as objects in the class. 
     * @param subsystem represents a subsystem. 
     */
    public SuperStructure(Autonomous auto, Swerve swerve, Vision vision, Aimbot aimbot, Intake intake, Hopper hopper, Shooter shooter, Zones zone){
        this.auto = auto;
        this.swerve = swerve;
        this.vision = vision;
        this.targeting = aimbot;
        this.intake = intake;
        this.hopper = hopper;
        this.shooter = shooter;

        DriverStation.silenceJoystickConnectionWarning(true);
        this.swerve.setJoysickInputs(() -> driverController.getLeftX(), () -> driverController.getLeftY(), () -> driverController.getRightX());
        this.swerve.setDriveFieldRelative(true);
        this.swerve.setSwerveState(SwerveState.manual);

        targeting.giveResources(operatorController);
    }

    /**
     * This method runs every iteration (every 20ms). Actions like state management and stateless logic are run here.
     */
    public void periodic(){
        /**
         * intake right trigger
         * score left trigger
         * pass left bumper
         */
        
        //Timer for the periodic
        long startTime = System.nanoTime();
        SmartDashboard.putString("SuperState", superState.name());

        SmartDashboard.putBoolean("roboPoseIsNull", vision.getRobotPose() == null);
        Pose3d visionRobotPose = vision.getRobotPose(vision.shooterCam, 0.1);
        if(visionRobotPose != null && RobotBase.isReal()){
            swerve.addVisionMeasurement(visionRobotPose.toPose2d());
        }

        // Driver controller commands
        /* Current plan for driver controls:
            Left Trigger: Intake
            Left Bumper: Intake with Object Detection
            Left Back: Retract Intake
            Right Trigger: Shoot in Hub (with aiming)
            Right Bumper: Pass/HubPreset
            Right Back: Align to hub preset (nearest of 5)
            A button: Bump Align
         * 
         */
        if(driverController.getStartButton()) {
            swerve.zeroGyro();
            swerve.resetOdometry(new Pose2d(0, 0, swerve.getYaw()));
        }
        if(operatorController.getStartButtonPressed()){
            if(superState != SuperState.manual)
                setSuperState(SuperState.manual);
            else 
                setSuperState(SuperState.deployed);
        }


        /*if(driverController.getLeftTriggerAxis() > 0.15 && !moveThroughTrench){
            moveThroughTrench = true;
            swerve.setPath(auto.getNearestTrenchPath(swerve.getPose()));
            swerve.setSwerveState(SwerveState.initializePath);
        }else if(driverController.getLeftTriggerAxis() <= 0.15 && moveThroughTrench){
            moveThroughTrench = false;
        }

        if(driverController.getLeftBumperButtonPressed()){
            orientWithBump = true;
            swerve.setSnapAngle(45);
            swerve.setSwerveState(SwerveState.snapAngle);
        }else if(driverController.getLeftBumperButtonReleased()){
            orientWithBump = false;
        }*/

        // Driver controller commands
        if(driverController.getLeftBumperButtonPressed()) {
            intaking = true;
            useObjectDetection = true;
        } else if(driverController.getLeftBumperButtonReleased()) {
            intaking = false;
            useObjectDetection = false;
        }
        if(driverController.getLeftTriggerAxis() > 0.1 && !intaking) {
            intaking = true;
        }else if(driverController.getLeftTriggerAxis() <= 0.1 && intaking){
            intaking = false;
        }
        if(driverController.getLeftStickButton()){
            intaking = false;
        }
        if(driverController.getRightBumperButtonPressed()) {
            passing = true;
            scoring = false;
        } else if(driverController.getRightBumperButtonReleased()) {
            passing = false;
        }
        if(driverController.getRightTriggerAxis() > 0.1 && !aimAtHub) {
            aimAtHub = true;
            scoring = true;
            passing = false;

            swerve.setSnapAngle(targeting.getCalculatedDirection());
            swerve.setSwerveState(SwerveState.snapAngle);
            scoringState = shotType.Parabolic;

        }else if(driverController.getRightTriggerAxis() <= 0.1 && aimAtHub){
            aimAtHub = false;
            scoring = false;
        }
        if(driverController.getRightStickButtonPressed()) {
            targeting.setPreset(presetShotType.Trench);
            targeting.setShotType(shotType.Preset);
            scoring = true;
        } else if(driverController.getRightStickButtonReleased()) {
            targeting.setShotType(shotType.Manual);
            scoring = false;
        }


        if(driverController.getAButtonPressed()) {
            orientWithBump = true;
        } else if(driverController.getAButtonReleased()) {
            orientWithBump = false;
        }
        if(driverController.getXButtonPressed()) {
            alignToHub = true;
            swerve.setPath(auto.getNearestHubScoringPath(swerve.getPose()));
            swerve.setSwerveState(SwerveState.initializePath);
        } else if(driverController.getXButtonReleased()) {
            alignToHub = false;
        }

        if(useObjectDetection){
            swerve.setGamepieceYaw(vision.getGamepieceYaw());
        }

        defaultDrive = !aimAtHub && !alignToHub && !moveThroughTrench && !orientWithBump && !useObjectDetection;

        if(defaultDrive && !inAuto){
            swerve.setSwerveState(SwerveState.manual);
        }


        // Operator controller commands
        if(operatorController.getStartButtonPressed()) {
            targeting.toggle();
        }
        if(operatorController.getAButtonPressed()) {
            currentShot = shotType.Direct;
        }
        if(operatorController.getBButtonPressed()) {
            currentShot = shotType.Parabolic;
        }
        if(operatorController.getXButtonPressed()) {
            currentShot = shotType.Apriltag;
        }
        if(operatorController.getBackButtonPressed()) {
            targeting.toggleVision();
        }

        //Operator state commands
        if(operatorController.getLeftBumperButton()) {
            passing = true;
        } else {
            passing = false;
        }

        if(operatorController.getRightTriggerAxis() > 0.15) {
            intaking = true;
        } else {
            intaking = false;
        }

        if(operatorController.getLeftTriggerAxis() > 0.15) {
            scoring = true;
        } else {
            scoring = false;
        }

        stow = operatorController.getBButton();

        hasAcheivedState = manageSuperState(superState);

        SmartDashboard.putNumber("ShooterAngle", targeting.getCalculatedAngle());
        SmartDashboard.putNumber("ShooterSpeed", targeting.getCalculatedSpeed());

        long endTime = System.nanoTime();

        //Timing measurement, in milliseconds
        SmartDashboard.putNumber("Time for superstructure periodic", (double) ((endTime - startTime) / 1000000.0));
    }

    /**
     * This method is run every iteration (20ms) only in simulation mode. Can be used to update simulated mechanisms or sensors.
     */
    public void simulationPeriodic(){}

    /**
     * This is called the state manager. It checks the value of the SuperState argument and calls the method associated with it.
     * <p>Each state will have its own case statement, returning its state method.
     * @param state the SuperState value
     * @return the condition of the state determined by the state method.
     */
    private boolean manageSuperState(SuperState state){
        switch(state){
            case stowed:
                return stowed();
            case deployed:
                return deployed();
            case score:
                return score();
            case intake:
                return intake();
            case preparePass:
                return preparePass();
            case pass:
                return pass();
            case snowBlowerPass:
                return snowBlowerPass();
            case snowBlowerScore:
                return snowBlowerScore();
            case prepareHubShot:
                return prepareHubShot();
            case manual:
                return manual();
            case autonomous:
                return autonomous();
            case autonomousEnter:
                return autonomousEnter();
            case autonomousExit:
                return autonomousExit();
            
            default:
                return defaultState(state);
        }
    }

    /**
     * Each of these methods, called "state methods", represent a defined state.
     * When called by the state manager, it will set the states of different subsystems.
     * @return true if the state is complete. The condition could represent mechanisms at a setpoint, a beambreak trigger, a timer, etc.
     * Mainly used for autonomous routines.
     */

    private boolean stowed(){
        targeting.setToggle(true);
        targeting.setShotType(shotType.Transport);
        shooter.setState(ShooterState.followAimbot);
        intake.setState(IntakeState.stow);
        hopper.setState(HopperState.idle);

        if(intaking) {
            setSuperState(SuperState.deployed);
        }
        return intake.isAtSetpoint() && shooter.isAtSetpoint();
    }

    private boolean deployed(){
        targeting.setToggle(true);
        targeting.setShotType(shotType.Transport);
        shooter.setState(ShooterState.followAimbot);
        if(intaking){
            setSuperState(SuperState.intake);
            return true;
        }

        if(hopper.getBeamBreak() || stow){
            setSuperState(SuperState.stowed);
            return true;
        }
        targeting.setShotType(shotType.Transport);
        
        intake.setState(IntakeState.deploy);
        hopper.setState(HopperState.idle);

        if(passing) {
            setSuperState(SuperState.preparePass);
        }
        else if(scoring) {
            setSuperState(SuperState.prepareHubShot);
        }

        return intake.isAtSetpoint() && shooter.isAtSetpoint();
    }

    private boolean intake(){
        targeting.setToggle(true);
        intake.setState(IntakeState.intake);
        hopper.setState(HopperState.intake);
        
        if(useObjectDetection){
            
        }

        if(!intaking) {
            setSuperState(SuperState.deployed);
        }

        if(passing) {
            setSuperState(SuperState.preparePass);
        } else if(scoring) {
            setSuperState(SuperState.prepareHubShot);
        }
        
        return intake.isAtSetpoint();
    }

    private boolean preparePass() {
        targeting.setToggle(true);
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            targeting.setPreset(presetShotType.RegHubShot);
            targeting.setShotType(shotType.Preset);
        }
        intake.setState(IntakeState.deploy);
        hopper.setState(HopperState.idle);
        if(shooter.isAtSetpoint() && shooter.isAtSpeedSetpoint()) {
            if(intaking) {
                setSuperState(SuperState.snowBlowerPass);
            }
            setSuperState(SuperState.pass);
            return true;
        }

        if(!passing) {
            setSuperState(SuperState.deployed);
        }
        return false;
    }

    private boolean pass(){
        targeting.setToggle(true);
        targeting.setPreset(presetShotType.RegHubShot);
        targeting.setShotType(shotType.Preset);
        shooter.setState(ShooterState.followAimbot);
        intake.setState(IntakeState.deploy);
        hopper.setState(HopperState.shoot);
        if(intaking){
            setSuperState(SuperState.snowBlowerPass);
        }
        if(!passing){
            setSuperState(SuperState.deployed);
        }
        return shooter.isAtSetpoint();
    }

    private boolean snowBlowerPass(){
        targeting.setToggle(true);
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            targeting.setPreset(presetShotType.Pass);
            targeting.setShotType(shotType.Preset);
        }
        intake.setState(IntakeState.intake);
        hopper.setState(HopperState.intake);
        if(!intaking){
            setSuperState(SuperState.pass);
        }

        if(!passing) {
            setSuperState(SuperState.deployed);
        }
        return shooter.isAtSetpoint();
    }

    private boolean prepareHubShot(){
        targeting.setToggle(true);
        ShooterState shooterState = ShooterState.followAimbot;
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            targeting.setShotType(scoringState);
        }
        shooter.setState(shooterState);

        if(shooter.isAtSetpoint() && shooter.isAtSpeedSetpoint()) {
            if(intaking) {
                setSuperState(SuperState.snowBlowerScore);
            }else
                setSuperState(SuperState.score);
            return true;
        }
        if(!scoring) {
            setSuperState(SuperState.deployed);
        }
        return false;
    }

    private boolean score(){
        targeting.setToggle(true);
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            targeting.setShotType(scoringState);
        }
        shooter.setState(ShooterState.followAimbot);
        
        if(intaking) {
            setSuperState(SuperState.snowBlowerScore);
        }
        if(!scoring) {
            setSuperState(SuperState.deployed);
        }

        return shooter.isAtSetpoint();
    }

    private boolean snowBlowerScore(){
        targeting.setToggle(true);
        intake.setState(IntakeState.intake);
        if(moveThroughTrench){
            targeting.setShotType(shotType.Transport);
        }else{
            targeting.setShotType(scoringState);
        }
        shooter.setState(ShooterState.followAimbot);
        hopper.setState(HopperState.shoot);
        if(!scoring) {
            setSuperState(SuperState.deployed);
        }
        if(!intaking) {
            setSuperState(SuperState.score);
        }
        
        return shooter.isAtSetpoint();
    }

    private boolean manual(){
        // code for direct control of mechanisms goes here
        targeting.setToggle(true);
        intake.setState(IntakeState.manual);
        targeting.setShotType(shotType.Manual);
        hopper.setState(HopperState.manual);  
        
        if(!operatorController.getAButton()){
            intake.setPosition(Math.abs(operatorController.getRightY() * 128));
        }else
            intake.setPosition(128);

        return true;
    }

    /** 
     * The state of the robot when autonomous initializes.
     * <p>It will reset/initialize certain variables and set the auto to the selected list of {@link AutoAction}'s from the auto chooser in {@link Autonomous}
     * before switching to the autonomous state. It also calls the autonomous state so it does not have to wait an extra iteration
     * @return true since there is no logic to compute
     */
    private boolean autonomousEnter(){
        setSuperState(SuperState.autonomous);
        autoIndex = 0;
        autoActions = auto.getAutonomous(auto.getSelectedAuto());
        autonomous();
        return true;
    }

    /**
     * The state of the robot during the full autonomous period.
     * This will run each {@link AutoAction} from the list received from {@link Autonomous} in order.
     * <p>If there is an alternative condition in the AutoAction, this method should increment the autoIndex variable after the condition is true.
     * <p>Otherwise, if the AutoAction is a {@link Path}, this method should increment the variable when the path is finished;
     * if it's a {@link SuperState}, this method should increment the variable when the SuperState's state method returns true 
     * @return true if autoIndex >= the size of the AutoAction List
     */
    private boolean autonomous(){
        inAuto = true;
        boolean isActionFinished = true;
        AutoAction autoAction;
        autoAction = autoActions.get(autoIndex);

        SmartDashboard.putNumber("AutoAction", autoActions.size() - 1);
        SmartDashboard.putNumber("autoIndex", autoIndex);
        SmartDashboard.putBoolean("IsPathFinished", isActionFinished);

        if (autoAction.isPath()){
            SwerveState swerveState = swerve.getSwerveState();
            if (!swerveState.equals(SwerveState.initializePath) && !swerveState.equals(SwerveState.runPath)){
                swerve.setPath(autoAction.getAsPath());
                swerve.setSwerveState(SwerveState.initializePath);
            }
            isActionFinished = autoAction.getAsPath().isPathFinished();
        } else if (autoAction.isState()) {
            isActionFinished = manageSuperState(autoAction.getAsState());
        }

        if (autoAction.hasAlternateCondition()){
            isActionFinished = autoAction.getAlternateCondition();
        }

        if (autoAction.hasAdditionalCondition()){
            isActionFinished = (isActionFinished && autoAction.getAdditionalCondition());
        }

        if (autoAction.hasOptionalCondition()){
            isActionFinished = (isActionFinished || autoAction.getOptionalCondition());
        }

        if (isActionFinished) {
            if (autoActions.size() - 1 > autoIndex) {
                autoIndex += 1;
                
            }
            else {
                inAuto = false;
                setSuperState(SuperState.autonomousExit);
            }

            
        }

        return true;
    }

    /**
     * The state of the robot at the end of the autonomous period.
     * <p>This will set the robot up for Teleop control by setting inAuto to false, 
     * setting the swerve state to manual, and changing the superState (using setSuperState()) to a different state
     * @return true 
     */
    private boolean autonomousExit(){
        inAuto = false;
        swerve.setSwerveState(SwerveState.manual);
        setSuperState(SuperState.deployed);
        
        //TODO: Code for setting up teleop goes here.
        // this may include setting swerve to manual control, setting the superState to a different state, etc.
        return true;
    }

    /**
     * The method called when the state method of the managed SuperState is not called.
     * @param state
     * @return
     */
    private boolean defaultState(SuperState state){
        System.err.println("The SuperState \"" + state.name() + "\" does not have a state method or it was not called."
            + "\nPlease return the state method in a new case statement in manageState()");
        return true;
    }

    /**
     * This method is responsible for changing the robot's {@link SuperState}.
     * <p> This method MUST be used when changing the robot's SuperState to ensure it does not interfere with autonomous.
     * @param state the SuperState to set the robot if not in Autonomous mode
     */
    public void setSuperState(SuperState state){
    
        if(!inAuto)
            superState = state;
    }

    /**
     * Gets the current {@link SuperState} of the robot.
     * @return the value of the superState object
     */
    public SuperState getSuperState(){
        return superState;
    }

    public void disableAuto(){
        superState = SuperState.autonomousExit;
    }
}
