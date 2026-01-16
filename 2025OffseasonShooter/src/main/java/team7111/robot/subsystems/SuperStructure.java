
package team7111.robot.subsystems;

import team7111.robot.subsystems.IntakeSubsystem.IntakeState;
import team7111.robot.subsystems.ShooterSubsystem.ShooterState;
import team7111.robot.subsystems.BarrelSubsystem.BarrelState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase{

    public enum ControlState {
        intakeTrigger,
        ejectTrigger,
        prepareShotTrigger,
        prepareShotVisionTrigger,
        shootTrigger,
        manualToggle,
    }

    private enum SuperState {
        intake,
        secure,
        shoot,
        prepareShot,
        prepareShotVision,
        eject,
        shootVision,
        manual,
        unloaded,
        loaded,
        autonomous,
        defaultState,
    }

    private VisionSubsystem vision;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private BarrelSubsystem barrel;
    private SwerveSubsystem swerve;
    private PathSubsystem paths;

    private boolean intakeTrigger = false;
    private boolean ejectTrigger = false;
    private boolean prepareShotTrigger = false;
    private boolean prepareShotVisionTrigger = false;
    private boolean shootTrigger = false;
    private boolean manualToggle = false;
    
    private SuperState superState = SuperState.unloaded;

    private double shotTimer = 0;
    private double ejectTimer = 0;
    private double secureTimer = 0;

    private XboxController operatorController;

    public SuperStructure(
        VisionSubsystem vision, SwerveSubsystem swerve, PathSubsystem paths, 
        IntakeSubsystem intake, BarrelSubsystem barrel, ShooterSubsystem shooter,
        int operatorPort
    ){
        this.vision = vision;
        this.swerve = swerve;
        this.paths = paths;
        this.intake = intake;
        this.barrel = barrel;
        this.shooter = shooter;

        this.operatorController = new XboxController(operatorPort);
    }

    public void periodic() {
        manageSuperState();
        SmartDashboard.putString("SuperState", superState.name());
        SmartDashboard.putBoolean("ManualToggle", manualToggle);
        if (manualToggle)
            setSuperState(SuperState.manual);
    }

    private void manageSuperState() {
        switch (superState) {
            case intake:
                intake();
                break;
            case secure:
                secure();
                break;
            case shootVision:
                shootVision();
                break;
            case eject:
                eject();
                break;
            case manual:
                manual();
                break;
            case defaultState:
                defaultState();
                break;   
            case shoot:
                shoot();
                break;
            case unloaded:
                unloaded();
                break;
            case prepareShot:
                prepareShot();
                break;
            case prepareShotVision:
                prepareShotVision();
                break;
            case loaded:
                loaded();
                break;
            case autonomous:
                autonomous();
                break;
            default:
                break;
        }
    }
    private void setControlState(ControlState button, boolean state) {
        switch (button) {
            case intakeTrigger:
                intakeTrigger = state;
                break;
            case ejectTrigger:
                ejectTrigger = state;
                break;
            case prepareShotTrigger:
                prepareShotTrigger = state;
                break;
            case prepareShotVisionTrigger:
                prepareShotVisionTrigger = state;
            case shootTrigger:
                shootTrigger = state;
                break;
            case manualToggle:
                manualToggle = state;
                break;
            default:
                break;
        }
    }

    public Command setControlStateCommand(ControlState button, boolean state){
        return runOnce(() -> setControlState(button, state));
    }

    public void setSuperState(SuperState state) {
        superState = state;
    }

    private void flipManual() {
        manualToggle = !manualToggle;
        
    }

    public Command flipManualCommand() {
        return runOnce(() -> flipManual());
    }

    private void unloaded() {
        // Sets intake to store, barrel to unload, and shooter to idle.
        intake.setState(IntakeState.store);
        barrel.setState(BarrelState.unload);
        shooter.setState(ShooterState.idle);
        if (intakeTrigger) {
            setSuperState(SuperState.intake);
        }
        
        else if(prepareShotVisionTrigger) {
            setSuperState(SuperState.prepareShotVision);
        }

    }

    private void intake() {
        // Sets intake to deploy, and barrel to intake. If beambreak is active, sets main state to secure
        intake.setState(IntakeState.deploy);
        barrel.setState(BarrelState.intake);
        shooter.setState(ShooterState.idle);
        if (barrel.getBeamBreak()) {
            setSuperState(SuperState.secure);
        }
        if (!intakeTrigger) {
            setSuperState(SuperState.unloaded);
        }
    }

    private void secure() {
        // Sets shooter to prepare shot, inatke to transition, and barrel to adjust. If beambreak is false, sets barrel to readjust. If beambreak active sets superstate to loaded
        shooter.setState(ShooterState.idle);
        intake.setState(IntakeState.transition);
        barrel.setState(BarrelState.loaded);
        secureTimer += 1;
        if (secureTimer >= 50) {
            setSuperState(SuperState.loaded);
        }
    }

    private void loaded() {
        // The state in which there is a gamepiece inside the barrel of the robot
        intake.setState(IntakeState.store);
        barrel.setState(BarrelState.loaded);
        shooter.setState(ShooterState.idle);
        if (ejectTrigger) {
            setSuperState(SuperState.eject);
        }
        if (prepareShotTrigger) {
            setSuperState(SuperState.prepareShot);
        }
    }

    private void eject() {
        // Sets the shooter state to shoot, to eject the game piece from intake and set superstate to unloaded
        intake.setState(IntakeState.eject);
        barrel.setState(BarrelState.reverse);
        shooter.setState(ShooterState.reverse);
        if (!barrel.getBeamBreak()) {
            ejectTimer += 1;
        }
        if (ejectTimer >= 150) {
            setSuperState(SuperState.unloaded);
            ejectTimer = 0;
        }
        if (!ejectTrigger) {
            setSuperState(SuperState.secure);
        }
    }

    private void prepareShot() {
        shooter.setState(ShooterState.prepareShot);
        intake.setState(IntakeState.store);
        barrel.setState(BarrelState.loaded);
        if (shootTrigger) {
            setSuperState(SuperState.shoot);
        }
    }

    private void prepareShotVision() {
        setSuperState(SuperState.shootVision);
    }

    private void shoot() {
        // Sets shooter, and barrel to shoot. Starts the shot timer. When shot timer ends, loops, and set Super state to unloaded
        shooter.setState(ShooterState.shoot);
        barrel.setState(BarrelState.shoot);
        intake.setState(IntakeState.store);
        if (barrel.getBeamBreak() || shotTimer >= 50) {
            shotTimer = 0;
        }
        if (!barrel.getBeamBreak()) {
            shotTimer += 1;
        }
        if (shotTimer >= 50) {
            setSuperState(SuperState.unloaded);
        }
    }

    private void shootVision() {
        // sets the shooter state to shoot. Will aim using vision
        shooter.setAngle(vision.shooterAngle());
        shooter.setState(ShooterState.shoot);
    }

    private void autonomous(){
        
    }

    private void manual() {
        // Sets shooter, and intake to manual for manual controlState
        intake.setState(IntakeState.manual);
        barrel.setState(BarrelState.manual);
        shooter.setState(ShooterState.manual);

        double intakeSpeed = operatorController.getLeftTriggerAxis();
        double chargeShotSpeed = operatorController.getRightTriggerAxis();
        boolean shoot = operatorController.getLeftBumperButton();

        // intake and barrel speeds are in duty cycle, shooter in RPM
        double manualIntakeSpeed = 0;
        double manualBarrelSpeed = 0;
        double manualShooterSpeed = 0;
        if (-operatorController.getLeftY() >= 0.2 || operatorController.getLeftY() >= 0.2)
        {
            shooter.addManualAngle(-operatorController.getLeftY() * 5);
        }
        if (-operatorController.getRightY() >= 0.2 || operatorController.getRightY() >= 0.2)
        {
            intake.addManualAngle(-operatorController.getRightY() * 5);
        }
        
        
        
        if(intakeSpeed > 0.05){
            manualIntakeSpeed = intakeSpeed;
            manualBarrelSpeed = intakeSpeed;
        }
        if(chargeShotSpeed > 0.05){
            manualShooterSpeed = chargeShotSpeed * 2000;
        }
        if(shoot){
            manualBarrelSpeed = 0.75;
        }
        intake.setManualSpeed(manualIntakeSpeed);
        barrel.setManualSpeed(manualBarrelSpeed);
        shooter.setManualSpeed(manualShooterSpeed);

        if (!manualToggle)
            setSuperState(SuperState.unloaded);
    }

    private void defaultState() {
        // Sets shooter, and intake to their default state, making them inactive
        shooter.setState(ShooterState.defaultState);
        intake.setState(IntakeState.defualtState);
        barrel.setState(BarrelState.defaultState);
    }
}