package team7111.robot.utils.motor;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import team7111.robot.utils.encoder.GenericEncoder;


public class REVMotor implements Motor {
    private SparkMax motor;
    private PIDController pid;
    private GenericEncoder encoder = null;
    private double gearRatio = 1;
    private SimpleMotorFeedforward feedForward;
    private double setPoint;
    private Motor simType;
    private double currentSetpoint; 
    private double positiveSpeedLimit = 1;
    private double negativeSpeedLimit = -1;
    
    public REVMotor (int id) {
        motor = new SparkMax(id, MotorType.kBrushless);

    }
    
    public REVMotor(int id, GenericEncoder encoder, double gearRatio, PIDController pid, SimpleMotorFeedforward feedforward, Motor simType, SparkBaseConfig sparkMotorConfig){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.feedForward = feedforward;

        motor = new SparkMax(id, MotorType.kBrushless);
        this.simType = simType;

        motor.configure(sparkMotorConfig, null, null);
        
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Voltage", () -> motor.getBusVoltage()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Speed", () -> motor.get()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("Motor" + id + " Position", () -> motor.getAbsoluteEncoder().getPosition()).withWidget("");
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
            motor.getEncoder().setPosition(position / gearRatio);
        }
    }
    
    public double getPosition(){
        if(encoder == null){
            return motor.getEncoder().getPosition() * gearRatio;
        } else{
            return encoder.getPosition().getDegrees();
        }
    }  
    
    public void setSetpoint(double setPoint, boolean useSimFF){
        double pidOutput = pid.calculate(getPosition(), setPoint);
        double feedforwardOutput = feedForward != null
            ? feedForward.calculate(pid.getErrorDerivative())
            : 0;
        double output = pidOutput + feedforwardOutput;

        this.setPoint = setPoint;
        if(output > positiveSpeedLimit){
            output = positiveSpeedLimit;
        }else if(output < negativeSpeedLimit){
            output = negativeSpeedLimit;
        }
        motor.setVoltage(pidOutput + feedforwardOutput); //Needs velocity for feedforward
        
    }

    public void periodic(){
        if (encoder != null){
            encoder.periodic();
        }
    }

    public void setPID(double p, double i, double d){
        pid.setPID(p, i, d);
    }

    public PIDController getPID(){
        return pid;
    }

    public GenericEncoder getEncoder(){
        return encoder;
    }

    public double getVoltage(){
        return motor.getBusVoltage();
    }

    public void setVoltage(double volts){
        motor.setVoltage(volts);
    }

    public boolean isAtSetpoint(double deadzone){
        if (getPosition() >= setPoint - deadzone && getPosition() <= setPoint + deadzone)
            return  true;
        else
            return false;
    }

    public SimpleMotorFeedforward getFeedForward(){
        return feedForward;
    }

    public void setFeedFoward(double kS, double kV, double kA){
        feedForward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public void setSpeedLimits(double positiveSpeedLimit, double negativeSpeedLimit){
        this.positiveSpeedLimit = positiveSpeedLimit;
        this.negativeSpeedLimit = negativeSpeedLimit;
    }
    
}
