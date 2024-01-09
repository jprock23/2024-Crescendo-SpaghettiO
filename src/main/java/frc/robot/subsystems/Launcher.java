package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Ports;

public class Launcher {

    double power = .5;

    public enum LauncherState {
        AMP(0.0),
        SPEAKER(0.0),
        HANDOFF(0.0),
        RETRACTED(0.0);

        private double position;

        private LauncherState(double position) {
            this.position = position;
        }
    }

    public enum LauncherVoltage {
        AMP(0.0),
        SPEAKER(0.0),
        HANDOFF(0.0),
        OFF(0.0);

        private double volts;

        private LauncherVoltage(double volts) {
            this.volts = volts;
        }
    }

    public enum LauncherControl {
        MANUAL,
        PID,
    }

    public enum FlickerState {
        OUT(0.0),
        IN(0.0);

        private double position;

        private FlickerState(double position){
            this.position = position;
        }
    }

    private boolean test = true;

    private CANSparkMax launchMotor1;
    private CANSparkMax launchMotor2;

    private CANSparkMax flicker;

    private CANSparkMax launcherAngle1;
    private CANSparkMax launcherAngle2;

    private SparkMaxPIDController anglePID1;
    private SparkMaxPIDController anglePID2;

    private SparkMaxPIDController launchPID1;
    private SparkMaxPIDController launchPID2;

    private SparkMaxPIDController flickerPID;

    private AbsoluteEncoder angleEncoder1;
    private AbsoluteEncoder angleEncoder2;


    private LauncherState launcherState = LauncherState.RETRACTED;
    private LauncherVoltage launcherVolts = LauncherVoltage.OFF;
    private LauncherControl controlMode = LauncherControl.PID;
    private FlickerState flickerState = FlickerState.IN;

    public static Launcher instance;

    public Launcher() {
        launchMotor1 = new CANSparkMax(Ports.flywheel1, MotorType.kBrushless);
        launchMotor1.setInverted(false);
        launchMotor1.burnFlash();

        launchMotor2 = new CANSparkMax(Ports.flywheel2, MotorType.kBrushless);
        launchMotor2.setInverted(true);
        launchMotor2.burnFlash();

        flicker = new CANSparkMax(Ports.ringPusher, MotorType.kBrushless);
        flicker.setInverted(false);
        flicker.burnFlash();

        launcherAngle1 = new CANSparkMax(Ports.launcherAngle1, MotorType.kBrushless);
        launcherAngle1.setInverted(false);
        launcherAngle1.burnFlash();

        launcherAngle2 = new CANSparkMax(Ports.launcherAngle2, MotorType.kBrushless);
        launcherAngle2.setInverted(true);
        launcherAngle2.burnFlash();

        launchPID1 = launchMotor1.getPIDController();
        launchPID2 = launchMotor2.getPIDController();

        launchPID1.setP(LauncherConstants.launchPCoefficient);
        launchPID1.setI(LauncherConstants.launchICoefficient);
        launchPID1.setD(LauncherConstants.launchDCoefficient);
         
        launchPID2.setP(LauncherConstants.launchPCoefficient);
        launchPID2.setI(LauncherConstants.launchICoefficient);
        launchPID2.setD(LauncherConstants.launchDCoefficient);

        launchPID1.setFeedbackDevice(launchMotor1.getEncoder());
        launchPID2.setFeedbackDevice(launchMotor2.getEncoder());

        anglePID1 = launcherAngle1.getPIDController();
        anglePID2 = launcherAngle2.getPIDController();

        angleEncoder1 = launcherAngle1.getAbsoluteEncoder(Type.kDutyCycle);
        angleEncoder2 = launcherAngle2.getAbsoluteEncoder(Type.kDutyCycle);

        anglePID1.setP(LauncherConstants.anglePCoefficient);
        anglePID1.setI(LauncherConstants.angleICoefficient);
        anglePID1.setD(LauncherConstants.angleDCoefficient);

        anglePID1.setP(LauncherConstants.anglePCoefficient);
        anglePID1.setI(LauncherConstants.angleICoefficient);
        anglePID1.setD(LauncherConstants.angleDCoefficient);

        anglePID1.setFeedbackDevice(angleEncoder1);
        anglePID2.setFeedbackDevice(angleEncoder2);

        flickerPID =flicker.getPIDController();
        flickerPID.setP(LauncherConstants.anglePCoefficient);
        flickerPID.setI(LauncherConstants.angleICoefficient);
        flickerPID.setD(LauncherConstants.angleDCoefficient);

        flickerPID.setFeedbackDevice(flicker.getEncoder());

    }

    public void periodic(){
        if (test){
            setLauncherPower();
        }

        anglePID1.setReference(launcherState.position,  CANSparkMax.ControlType.kPosition);
        anglePID2.setReference(launcherState.position,  CANSparkMax.ControlType.kPosition);

        launchPID1.setReference(launcherVolts.volts, CANSparkMax.ControlType.kVoltage);
        launchPID2.setReference(launcherVolts.volts, CANSparkMax.ControlType.kVoltage);

    }

    public void setLauncherVolts(LauncherVoltage state){
        this.launcherVolts = state;
    }

    public void setLauncherPower() {
        launchMotor1.set(power);
        launchMotor2.set(power);
    }

    public void increasePower(){
        power += .1;
    }

    public void decreasePower(){
        power -= .1;
    }

    public double getPower(){
        return power;
    }

    public double getLauncherPosition() {
        return (angleEncoder1.getPosition() + angleEncoder2.getPosition())/2;
    }


    public boolean hasReachedPose(double tolerance) {
        if (Math.abs(getLauncherPosition() - launcherState.position) > tolerance) {
            return true;
        }
            return false;
    }

    public void setTest(){
        test = !test;
    }

    public void setFlickerState(FlickerState state){
        flickerState = state;
    }

    public void setLauncherState(LauncherState state) {
        launcherState = state;
    }

    public static Launcher getInstance() {
        if(instance == null)
            instance = new Launcher();
        return instance;
    }
}
