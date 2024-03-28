package frc.robot.subsystems.launcher;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.IO.DigitalInputs;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.Ports;

public class Launcher {

    public enum LauncherState {
        AMP(-49, 0.9),
        START(0, 0.0),
        TRAP(-70.04991149902344, 0.8),
        LONG(-13.25, 1.0),
        HANDOFF(9,0.5),
        HOVER(-3, 1.0), 
        TOSS(-18, .9),
        AUTOMIDSHOT(-13.75, 1.0),
        //height: ?
        AUTOLEFTSHOT(-13.5, 1.0),
        //height: 20.75
        AUTORIGHTSHOT(-13.5, 1.0),
        //height: ?7
        SPEAKER(-59.0, 1.0),
        INTERLOPE(0.0, 1.0),
        TEST(-13.25, 1.0);

        public double position;
        public double launchSpeed;

        private LauncherState(double position, double launchSpeed) {
            this.position = position;
            this.launchSpeed = launchSpeed;
        }
    }

    double anglePower = 0.2;

    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;
    private CANSparkMax sushiMotor;

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor;

    private double increment = 1.0;

    private ArmFeedforward feedForward;
    private SparkMaxPIDController pivotController1;

    private static RelativeEncoder encoder;
    private static AbsoluteEncoder absEncoder;

    private DigitalInputs breakBeam;

    private boolean[] connections = new boolean[8];

    private static LauncherState launchState = LauncherState.START;

    public static Launcher instance;

    public boolean ansh;

    public Launcher() {
        shootMotor1 = new CANSparkMax(Ports.shootMotor1, MotorType.kBrushless);
        shootMotor1.restoreFactoryDefaults();

        shootMotor1.setSmartCurrentLimit(60);
        shootMotor1.setIdleMode(IdleMode.kCoast);
        shootMotor1.setInverted(false);
        shootMotor1.burnFlash();

        shootMotor2 = new CANSparkMax(Ports.shootMotor2, MotorType.kBrushless);
        shootMotor2.restoreFactoryDefaults();

        shootMotor2.setSmartCurrentLimit(60);
        shootMotor2.setIdleMode(IdleMode.kCoast);
        shootMotor2.setInverted(false);
        shootMotor2.burnFlash();

        flicker = new CANSparkMax(Ports.flicker, MotorType.kBrushless);
        flicker.restoreFactoryDefaults();

        flicker.setSmartCurrentLimit(20);
        flicker.setIdleMode(IdleMode.kBrake);
        flicker.setInverted(false);
        flicker.burnFlash();

        pivotMotor = new CANSparkMax(Ports.pivotMotor, MotorType.kBrushless);
        pivotMotor.restoreFactoryDefaults();

        pivotMotor.setSmartCurrentLimit(60);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setInverted(true);
        // pivotMotor.setInverted(false);

        pivotMotor.setOpenLoopRampRate(1);

        sushiMotor = new CANSparkMax(Ports.sushi, MotorType.kBrushless);
        sushiMotor.restoreFactoryDefaults();

        sushiMotor.setSmartCurrentLimit(40);
        sushiMotor.setIdleMode(IdleMode.kCoast);
        sushiMotor.burnFlash();

        feedForward = new ArmFeedforward(0.012, 0.017, 0.0, 0.0);
        //u:.023 l:.011 mid:.017 ks:.012

        // Prototype numbers
        // upper: .045 lower: .0285 ks:.0085 kg:.037

        encoder = pivotMotor.getEncoder();
        // absEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        // absEncoder.setInverted(true);

        // absEncoder.setPositionConversionFactor(197.763157);

        pivotController1 = pivotMotor.getPIDController();

        pivotController1.setP(LauncherConstants.pivotPCoefficient);
        pivotController1.setI(LauncherConstants.pivotICoefficient);
        pivotController1.setD(LauncherConstants.pivotDCoefficient);

        pivotController1.setFeedbackDevice(encoder);
        // pivotController1.setFeedbackDevice(absEncoder);

        pivotController1.setOutputRange(-1, 1);

        // pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);

        pivotMotor.burnFlash();

        breakBeam = DigitalInputs.getInstance();

    }

    public void updatePose() {

        pivotController1.setReference(launchState.position, CANSparkMax.ControlType.kPosition, 0,
                feedForward.calculate(encoder.getPosition(), 0));

        // pivotController1.setReference(15, CANSparkMax.ControlType.kPosition, 0,
        // feedForward.calculate(encoder.getPosition(), 0));

        // pivotController1.setReference(20, ControlType.kPosition, 0,
        // feedForward.calculate(absEncoder.getPosition()* (2* Math.PI) + .349, 0));

        // pivotMotor.set(feedForward.calculate(absEncoder.getPosition()* (2* Math.PI) + .349, 0, 0));

    }

    public void interpolateAngle(){
        double delta = Drivebase.getStaticPose().getX();
        double position;

        position = -28.7958/Math.pow(delta, .70833333);

        LauncherState.INTERLOPE.position = MathUtil.clamp(position, LauncherState.SPEAKER.position, LauncherState.HOVER.position);
        // setLauncherState(LauncherState.INTERLOPE);
    }

    public void setPivotPower() {
        // pivotMotor.set(anglePower + feedForward.calculate(encoder.getPosition(), 0));
                pivotMotor.set(anglePower + feedForward.calculate(absEncoder.getPosition(), 0));

    }

    public void setReversePivotPower() {
        // pivotMotor.set(-anglePower + feedForward.calculate(encoder.getPosition(), 0));
            pivotMotor.set(anglePower + feedForward.calculate(absEncoder.getPosition(), 0));
    }


    public void eject(){
        shootMotor1.set(launchState.launchSpeed/4);
        shootMotor2.set(launchState.launchSpeed/4);
    }

    public void setPivotOff() {
        pivotMotor.set(0.0);
    }

    public double getTestPosition(){
        return LauncherState.INTERLOPE.position;
    }

    public void setLauncherOn() {
        if(launchState == LauncherState.AMP){
        shootMotor1.set(launchState.launchSpeed);
        shootMotor2.set(launchState.launchSpeed/2);
        } else {
        shootMotor1.set(launchState.launchSpeed);
        shootMotor2.set(launchState.launchSpeed);
        }
    }

    public void setReverseLauncherOn() {

        // if(launchState == LauncherState.AMP){
        // shootMotor1.set(-launchState.launchSpeed);
        // shootMotor2.set(launchState.launchSpeed * 0.36);
        // } else {
        shootMotor1.set(-launchState.launchSpeed);
        shootMotor2.set(-launchState.launchSpeed);
        // }
    }

    public void setLauncherOff() {
        shootMotor1.set(0.0);
        shootMotor2.set(0.0);
    }

    public void setFlickerOn() {
        flicker.set(1.0);
    }

    public void setFlickerReverse() {
        flicker.set(-1.0);
    }

    public void setFlickOff() {
        flicker.set(0);
    }

    public void setSushiOn(){
        sushiMotor.set(1);
    }

    public void setSushiReverse(){
        sushiMotor.set(-1);
    }

    public void setSushiOff(){
        sushiMotor.set(0);
    }

    // public double getRelativePosition(){
    //     return encoder.getPosition() + 16.4;
    // }

    public double getPosition() {
                return encoder.getPosition();

        // return absEncoder.getPosition();
    }

    public boolean getBreakBeam() {
        return !breakBeam.getInputs()[Ports.launcherBreakBeam];
    }

    public LauncherState getLaunchState() {
        return launchState;
    }

    public double getPivotCurrent() {
        return pivotMotor.getOutputCurrent();
    }

    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getPosition() - launchState.position) < tolerance;
    }

    public void setLauncherState(LauncherState state) {
        launchState = state;
    }

    public void increaseIncrement(){
        increment += 0.5;
    }

    public void decreaseInrement(){
        increment -= 0.5;
    }

    public void increasePosition(){
        LauncherState.TEST.position = LauncherState.TEST.position + increment;
    }

    
    public void decreasePosition(){
        LauncherState.TEST.position = LauncherState.TEST.position - increment;
    }

    public boolean[] launcherConnections() {

        if (shootMotor1.getBusVoltage() != 0) {
            connections[0] = true;
        } else {
            connections[0] = false;
        }

        if (shootMotor1.getOutputCurrent() != 0) {
            connections[1] = true;
        } else {
            connections[1] = false;
        }

        if (shootMotor2.getBusVoltage() != 0) {
            connections[2] = true;
        } else {
            connections[2] = false;
        }

        if (shootMotor2.getOutputCurrent() != 0) {
            connections[3] = true;
        } else {
            connections[3] = false;
        }

        if (pivotMotor.getBusVoltage() != 0) {
            connections[4] = true;
        } else {
            connections[4] = false;
        }

        if (pivotMotor.getOutputCurrent() != 0) {
            connections[5] = true;
        } else {
            connections[5] = false;
        }

        if (flicker.getBusVoltage() != 0) {
            connections[6] = true;
        } else {
            connections[6] = false;
        }

        if (flicker.getOutputCurrent() != 0) {
            connections[7] = true;
        } else {
            connections[7] = false;
        }

        return connections;
    }

    public boolean hasBrownedOut(){
        return pivotMotor.getFault(FaultID.kBrownout);
    }

    public void printConnections() {
        SmartDashboard.putBoolean("shootMotor1 Voltage", connections[0]);
        SmartDashboard.putBoolean("shootMotor1 Current", connections[1]);

        SmartDashboard.putBoolean("shootMotor2 Voltage", connections[2]);
        SmartDashboard.putBoolean("shootMotor2 Current", connections[3]);

        SmartDashboard.putBoolean("Pivot Voltage", connections[4]);
        SmartDashboard.putBoolean("Pivot Current", connections[5]);

        SmartDashboard.putBoolean("Flicker Voltage", connections[6]);
        SmartDashboard.putBoolean("Flicker Current", connections[7]);
    }

    public static Launcher getInstance() {
        if (instance == null)
            instance = new Launcher();
        return instance;
    }
}
