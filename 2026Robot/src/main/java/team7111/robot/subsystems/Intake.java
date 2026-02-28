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
import team7111.robot.utils.motor.TwoMotors;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Intake extends SubsystemBase {
    
    private Mechanism2d mechanism2d = new Mechanism2d(1, 1);
    private MechanismLigament2d intakeLigament = new MechanismLigament2d("Arm", 0.5, 37, 6, new Color8Bit(Color.kBlue));


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

    /** Maximum position in degrees */
    private final double maxPivotPos = 128;

    /** Minimum position in degrees */
    private final double minPivotPos = 0;

    private MotorConfig pivotConfig = new MotorConfig(20, 20, false, false, new PIDController(0.07, 0.0, 0.001), MechanismType.arm, 0, 0, 0, 0);
    private int pivotID = 12;

    private MotorConfig flyWheelConfig = new MotorConfig(1, 20, false, false, new PIDController(1, 0, 0), MechanismType.flywheel, 0, 0, 0, 0);
    private int flywheelLeadID = 10;
    private int flywheelFollowID = 11;

    private Motor pivot;

    private Motor flyWheel;

    private IntakeState currentState = IntakeState.stow;

    public Intake() {
        //TODO set CTRE motor ID to a real ID
        pivot = RobotBase.isReal()
            ? new REVMotor(pivotID, null, pivotConfig)
            : new ArmSimMotor(
                null,
                new SingleJointedArmSim(
                    DCMotor.getNEO(1), pivotConfig.gearRatio, 0.10849, 0.2, 
                    Degrees.of(minPivotPos).in(Radians), Degrees.of(maxPivotPos).in(Radians), true, Degrees.of(minPivotPos).in(Radians)), 
                pivotConfig.pid, 
                pivotConfig.armFF);

        //TODO set REV motor ID to a real ID
        flyWheel = RobotBase.isReal()
            ? new TwoMotors(new REVMotor(flywheelLeadID, null, flyWheelConfig), new REVMotor(flywheelFollowID, null, flyWheelConfig), flywheelLeadID, false)
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.01, flyWheelConfig.gearRatio), DCMotor.getNEO(2), 0.1),
                flyWheelConfig.pid,
                flyWheelConfig.simpleFF);
        
        mechanism2d.getRoot("Intake", 0.7, 0.3).append(intakeLigament);
        Shuffleboard.getTab("Mechanisms").add("intake", mechanism2d);
    }

    public void periodic(){
        manageState();

        if(pivotPos >= maxPivotPos) {
            pivotPos = maxPivotPos;
        }

        if(pivotPos <= minPivotPos) {
            pivotPos = minPivotPos;
        }
        //pivot.setSetpoint(pivotPos, false);
        //flyWheel.setDutyCycle(flyWheelSpeed);
        //flyWheel.periodic();

        
        //pivot.periodic();
        intakeLigament.setAngle(-pivot.getPosition() + maxPivotPos);
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
        pivotPos = minPivotPos;
        
    }

    private void deploy(){
        pivotPos = maxPivotPos;
    }

    private void intake(){
        pivotPos = maxPivotPos;
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

    public double getPosition(){
        return pivot.getPosition();
    }
    public double getSetpoint(){
        return pivotPos;
    }

    public void setPosition(double pos){
        pivotPos = pos;
    }
}
