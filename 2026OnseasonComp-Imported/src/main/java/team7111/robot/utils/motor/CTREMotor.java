package team7111.robot.utils.motor;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team7111.robot.utils.encoder.GenericEncoder;

public class CTREMotor implements Motor {
    private TalonFX motor;
    private PIDController pid = new PIDController(0.05, 0, 0);
    private TalonFXConfiguration config;
    private GenericEncoder encoder = null;
    private double gearRatio;
    private double currentSetpoint;
    private double positiveSpeedLimit = 1;
    private double negativeSpeedLimit = -1;
    private SimpleMotorFeedforward feedforward;
    private ArmFeedforward armFF;
    private Motor simType;
    private int id;

    private GenericEntry motorPEntry;
    private GenericEntry motorIEntry;
    private GenericEntry motorDEntry;
    
    public CTREMotor(int id, GenericEncoder encoder, double gearRatio, PIDController pid, SimpleMotorFeedforward feedForward, ArmFeedforward armFF, Motor simType, TalonFXConfiguration talonConfig){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.feedforward = feedForward;
        this.armFF = armFF;
        this.id = id;
        motor = new TalonFX(id);
        this.simType = simType;
        motorPEntry = Shuffleboard.getTab("test").add("Motor " + id + " P", 0).getEntry();
        motorIEntry = Shuffleboard.getTab("test").add("Motor " + id + " I", 0).getEntry();
        motorDEntry = Shuffleboard.getTab("test").add("Motor " + id + " D", 0).getEntry();

        motor.getConfigurator().apply(talonConfig);
        setPosition(0);

        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Voltage", () -> motor.getMotorVoltage().getValueAsDouble()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Speed", () -> motor.get()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Position", () -> 360 * motor.getRotorPosition().getValueAsDouble()).withWidget("");
    }

    public CTREMotor(int id){
        this.id = id;
        motor = new TalonFX(id);
        motorPEntry = Shuffleboard.getTab("test").add("Motor " + id + " P", 0).getEntry();
        motorIEntry = Shuffleboard.getTab("test").add("Motor " + id + " I", 0).getEntry();
        motorDEntry = Shuffleboard.getTab("test").add("Motor " + id + " D", 0).getEntry();

        setPosition(0);
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }

    public double getSpeed(){
        return motor.get();
    }
    
    public void setPosition(double position){
        if(encoder != null){
            encoder.setPosition(Rotation2d.fromDegrees(position));
        } else {
            motor.setPosition(position * gearRatio);
        }
    }

    
    public double getPosition(){
        if(encoder == null){
            return motor.getPosition().getValueAsDouble() / gearRatio;
        } else{
            return encoder.getPosition().getRotations();
        }
    }
        
    
    public void setSetpoint(double setPoint, boolean useSimFF){
        currentSetpoint = setPoint;
        double pidOutput = pid.calculate(getPosition(), setPoint);
        double feedforwardOutput;
        if(feedforward != null){
            feedforwardOutput = useSimFF
            ? armFF.calculate((setPoint + 0.1973) * 2 * Math.PI , pid.getErrorDerivative())
            : feedforward.calculate(pid.getErrorDerivative());
        }else 
            feedforwardOutput = 0;
        double totalOutput = pidOutput + feedforwardOutput;
        SmartDashboard.putNumber("Motor " + id + " pid", totalOutput);
        if(totalOutput > positiveSpeedLimit){
            motor.setVoltage(positiveSpeedLimit);
            //motor.set(positiveSpeedLimit);
        }else if(totalOutput < negativeSpeedLimit){
            motor.setVoltage(negativeSpeedLimit);
            //motor.set(negativeSpeedLimit);
        }else{
            motor.setVoltage(totalOutput);
        }
        
    }

    public void setPID(double p, double i, double d){
        pid.setPID(p, i, d);
    }

    public PIDController getPID() {
        return pid;
    }

    public GenericEncoder getEncoder(){
        return encoder;
    }

    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }
    
    public void setVoltage(double volts){
        motor.setVoltage(volts);
    }
    
    public boolean isAtSetpoint(double deadzone){
        if(getPosition() >= currentSetpoint - deadzone && getPosition() <= currentSetpoint + deadzone){
            return true;
        }
        return false;
    }
        
    public SimpleMotorFeedforward getFeedForward(){
        return feedforward;
    }

    public void setFeedFoward(double kS, double kV, double kA){
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        motor.getConfigurator().apply(config);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public void periodic(){
        if (encoder != null){
            encoder.periodic();
        }
        SmartDashboard.putNumber("Motor " + id + " setpoint", currentSetpoint);
        SmartDashboard.putBoolean("Motor " + id + " isAtSetpoint", isAtSetpoint(0.05));

        /*pid = new PIDController(
            motorPEntry.getDouble(0), 
            motorIEntry.getDouble(0), 
            motorDEntry.getDouble(0));*/
    }

    public void setSpeedLimits(double positiveSpeed, double negativeSpeed) {
        positiveSpeedLimit = positiveSpeed;
        negativeSpeedLimit = negativeSpeed;
    }
}
