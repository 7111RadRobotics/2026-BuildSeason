package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.utils.encoder.RelativeThroughBore;
import team7111.robot.utils.motor.ArmSimMotor;
import team7111.robot.utils.motor.CTREMotor;
import team7111.robot.utils.motor.FlywheelSimMotor;
import team7111.robot.utils.motor.Motor;
import team7111.robot.utils.motor.MotorConfig;
import team7111.robot.utils.motor.REVMotor;
import team7111.robot.utils.motor.TwoMotors;
import team7111.robot.utils.motor.Motor.MechanismType;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Shooter extends SubsystemBase {
    
    public enum ShooterState {
        idle,
        score,
        scoreAimbot,
        pass,
        stopped,
        manual,
    }

    private Mechanism2d mechanism2d = new Mechanism2d(3, 3);
    private MechanismLigament2d hoodLigament = new MechanismLigament2d("Shooter Hood", 1.5, 37);

    private MotorConfig hoodConfig = new MotorConfig(
        48/12 * 24/15 * 210/12, true, true, new PIDController(0.1, 0, 0), MechanismType.arm, 0.001, 0, 0, 0);

    private MotorConfig flywheelConfig = new MotorConfig(
        1, false, false, new PIDController(0, 0, 0), MechanismType.flywheel, 0.001, 0, 0, 0);

    private Motor hood;
    private Motor flywheels;

    private double hoodPosition = 37;
    private double flywheelSpeed = 0;

    private ShooterState currentState = ShooterState.stopped;

    public Shooter() {
        hood = RobotBase.isReal()
            ? new CTREMotor(1, new RelativeThroughBore(0, 0, 1), hoodConfig)
            : new ArmSimMotor(
                null,
                new SingleJointedArmSim(
                    DCMotor.getKrakenX60(1), hoodConfig.gearRatio, 0.01, 0.2, 
                    Degrees.of(37).in(Radians), Degrees.of(67).in(Radians), true, Degrees.of(37).in(Radians)), 
                hoodConfig.pid, 
                hoodConfig.armFF);
        
        flywheels = RobotBase.isReal()
            ? new TwoMotors(
                new REVMotor(0, null, flywheelConfig), 
                new REVMotor(0, null, flywheelConfig))
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.01, 1), DCMotor.getNEO(2), 0.1),
                flywheelConfig.pid,
                flywheelConfig.simpleFF);

        mechanism2d.getRoot("Shooter hood", 1.5, 1.5).append(hoodLigament);

        Shuffleboard.getTab("Mechanisms").add("Shooter", mechanism2d);
    }

    public void periodic(){
        manageState();
        hood.periodic();
        flywheels.periodic();

        hood.setSetpoint(90 - hoodPosition, false);
        flywheels.setVelocity(flywheelSpeed);

        hoodLigament.setAngle(hood.getPosition() + 90);
        SmartDashboard.putNumber("hood position", hood.getPosition());
    }

    public void simulationPeriodic(){}

    // The below methods are examples of retrieveable boolean values.
    // These can be checked in SuperStructure to determine a SuperState
    // or change a state/value in another subsystem.
    public boolean isAtSetpoint(){
        boolean isAtSetpoint = hood.isAtSetpoint(0.1); 
        return isAtSetpoint;
    }

    /**
     * This is the subsystem's state manager.
     * It calls the state method of the variable representing the subsystem's state.
     */
    private void manageState(){
        switch(currentState){
            case idle:
                idleMode();
                break;
            case manual:
                manual();
                break;
            case pass:
                pass();
                break;
            case score:
                score();
                break;
            case scoreAimbot:
                scoreAimbot();
                break;
            case stopped:
                stopped();
                break;
            default:
                break;
        }
    }

    private void idleMode(){
        hoodPosition = 50;
        flywheelSpeed = 1000;
    }

    private void pass(){}

    private void score(){}

    private void scoreAimbot(){}

    private void stopped(){
        hoodPosition = 30;
        flywheelSpeed = 0;
    }

    private void manual(){
        //System.out.println("Runs code for the manual state");
    }

    public void setState(ShooterState state){
        this.currentState = state;
    }

    public ShooterState getState(){
        return currentState;
    }
}
