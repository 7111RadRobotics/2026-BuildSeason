package team7111.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.utils.motor.MotorConfig;
import team7111.robot.utils.motor.REVMotor;
import team7111.robot.utils.motor.ArmSimMotor;
import team7111.robot.utils.motor.CTREMotor;
import team7111.robot.utils.motor.FlywheelSimMotor;
import team7111.robot.utils.motor.Motor.MechanismType;
import team7111.robot.utils.motor.Motor;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Intake extends SubsystemBase {
    
    private Mechanism2d mechanism2d = new Mechanism2d(1, 1);
    private MechanismLigament2d intakeLigament = new MechanismLigament2d("Arm", 1.5, 37, 6, new Color8Bit(Color.kOrange));


    /**
     * The enum that holds the values of the subsystem's states.
     * It's name should be the subsystem's followed by "State"
     */
    public enum IntakeState {
        stow,
        deploy,
        intake,
        manual,
    }

    private double flyWheelSpeed = 0;

    private double pivotPos = 0;

    private MotorConfig pivotConfig = new MotorConfig(12, false, false, new PIDController(0.15500259, 0.039, 0.0011), MechanismType.arm, 0, 0, 0, 0);

    private MotorConfig flyWheelConfig = new MotorConfig(1, false, false, new PIDController(1, 0, 0), MechanismType.flywheel, 0, 0, 0, 0);

    private Motor pivot;

    private Motor flyWheel;

    private IntakeState currentState = IntakeState.stow;

    public Intake() {
        //TODO set CTRE motor ID to a real ID
        pivot = RobotBase.isReal()
            ? new CTREMotor(21, null, pivotConfig)
            : new ArmSimMotor(
                null,
                new SingleJointedArmSim(
                    DCMotor.getKrakenX60(1), pivotConfig.gearRatio, 0.01, 0.2, 
                    Degrees.of(0).in(Radians), Degrees.of(90).in(Radians), true, Degrees.of(37).in(Radians)), 
                pivotConfig.pid, 
                pivotConfig.armFF);

        //TODO set REV motor ID to a real ID
        flyWheel = RobotBase.isReal()
            ? null //new REVMotor(20, null, flyWheelConfig)
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, flyWheelConfig.gearRatio), DCMotor.getNEO(1), 0.1),
                flyWheelConfig.pid,
                flyWheelConfig.simpleFF);
        
        mechanism2d.getRoot("Intake", 0.5, 0.5).append(intakeLigament);
        Shuffleboard.getTab("Mechanisms").add("intake", mechanism2d);
    }

    public void periodic(){
        manageState();
        pivot.setSetpoint(pivotPos, false);
        //flyWheel.setDutyCycle(flyWheelSpeed);
        //flyWheel.periodic();
        pivot.periodic();
        intakeLigament.setAngle(pivot.getPosition());
    }

    public void simulationPeriodic(){}

    // The below methods are examples of retrieveable boolean values.
    // These can be checked in SuperStructure to determine a SuperState
    // or change a state/value in another subsystem.
    public boolean isAtSetpoint(){
        boolean isAtSetpoint = true; // would be true if mechanisms were at/near their setpoints.
        return isAtSetpoint;
    }

    /**
     * This is the subsystem's state manager.
     * It calls the state method of the variable representing the subsystem's state.
     */
    private void manageState(){
        switch(currentState){
            case stow:
                stow();
                break;
            case deploy:
                deploy();
                break;
            case intake:
                intake();
                break;
            case manual:
                manual();
                break;
            default:
                break;
        }
    }

    private void stow(){
        //System.out.println("Runs code for the stow state");
        pivotPos = 45;
    }

    private void deploy(){
        //System.out.println("Runs code for the deploy state");
    }

    private void intake(){
        pivotPos = 5;
    }

    private void manual(){
        //System.out.println("Runs code for the manual state");
    }

    public void setState(IntakeState state){
        this.currentState = state;
    }

    public IntakeState getState(){
        return currentState;
    }
}
