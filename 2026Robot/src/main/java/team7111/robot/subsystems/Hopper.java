package team7111.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.utils.motor.FlywheelSimMotor;
import team7111.robot.utils.motor.Motor;
import team7111.robot.utils.motor.Motor.MechanismType;
import team7111.robot.utils.motor.MotorConfig;
import team7111.robot.utils.motor.REVMotor;
import team7111.robot.utils.motor.TwoMotors;
import team7111.robot.utils.motor.CTREMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * This class is an example to how a subsystem looks and functions.
 * The name of the file and class should be what it controls
 */
public class Hopper extends SubsystemBase {
    
    /**
     * The enum that holds the values of the subsystem's states.
     * It's name should be the subsystem's followed by "State"
     */
    public enum HopperState {
        intake,
        shoot,
        idle,
        manual,
    }

    private HopperState currentState = HopperState.idle;

    private MotorConfig spindexerConfig = new MotorConfig(
        1, 20, false, false, new PIDController(1, 0, 0), MechanismType.flywheel, 0.001, 0, 0, 0
    );
    private MotorConfig shooterIndexerConfig = new MotorConfig(
        1, 20, false, false, new PIDController(1, 0, 0), MechanismType.flywheel, 0.001, 0, 0, 0
    );

    private Motor spindexer;
    private Motor shooterIndexer;

    private double spindexerSpeed = 0;
    private double shooterIndexerSpeed = 0;

    public Hopper() {
        spindexer = RobotBase.isReal()
            ? new TwoMotors(new CTREMotor(13, null, spindexerConfig), new CTREMotor(14, null, spindexerConfig), 0, false)
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 0.01, spindexerConfig.gearRatio), DCMotor.getNEO(1), 0.1),
                spindexerConfig.pid,
                spindexerConfig.simpleFF
            );
        shooterIndexer = RobotBase.isReal()
            ? new REVMotor(15, null, shooterIndexerConfig)
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, shooterIndexerConfig.gearRatio), DCMotor.getNEO(1), 0.1),
                shooterIndexerConfig.pid,
                shooterIndexerConfig.simpleFF
            );
    }

    public void periodic(){
        manageState();

        spindexer.setDutyCycle(spindexerSpeed);
        shooterIndexer.setDutyCycle(shooterIndexerSpeed);
    }

    public void simulationPeriodic(){

    }

    public boolean getBeamBreak(){
        //TODO add beam break object to hopper and return it's value here
        return false;
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
            case intake:
                intake();
                break;
            case shoot:
                shoot();
                break;
            case manual:
                manual();
                break;
            default:
                break;
        }
    }

    // named differently to not overide a different method
    private void idleMode(){
        shooterIndexerSpeed = 0;
        spindexerSpeed = 0;
    }

    private void intake(){
        shooterIndexerSpeed = 0;
        spindexerSpeed = 0.0;
    }

    private void shoot(){
        shooterIndexerSpeed = 0.8;
        spindexerSpeed = 0.8;
    }

    private void manual(){
        //System.out.println("Runs code for the manual state");
    }

    public void setState(HopperState state){
        this.currentState = state;
    }

    public HopperState getState(){
        return currentState;
    }

    public void setSpeed(double speed){
        spindexerSpeed = speed;
        shooterIndexerSpeed = speed;
    }

    public double getSpeed(){
        return spindexer.getDutyCycle();
    }
}
