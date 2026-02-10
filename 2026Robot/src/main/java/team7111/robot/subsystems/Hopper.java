package team7111.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team7111.robot.utils.motor.FlywheelSimMotor;
import team7111.robot.utils.motor.Motor;
import team7111.robot.utils.motor.Motor.MechanismType;
import team7111.robot.utils.motor.MotorConfig;
import team7111.robot.utils.motor.REVMotor;
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

    private MotorConfig hopperMotorConfig = new MotorConfig(
        1, false, false, new PIDController(1, 0, 0), MechanismType.flywheel, 0.001, 0, 0, 0
    );

    private Motor hopperMotor;

    public Hopper() {
        hopperMotor = RobotBase.isReal()
            ? null //new REVMotor(30, null, hopperMotorConfig)
            : new FlywheelSimMotor(
                null, 
                new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, hopperMotorConfig.gearRatio), DCMotor.getNEO(1), 0.1),
                hopperMotorConfig.pid,
                hopperMotorConfig.simpleFF
            );
    }

    public void periodic(){
        manageState();
    }

    public void simulationPeriodic(){

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

    }

    private void intake(){

    }

    private void shoot(){

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
}
