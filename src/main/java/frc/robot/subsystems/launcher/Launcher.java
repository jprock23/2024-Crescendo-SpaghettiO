package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Ports;

public class Launcher {

    public enum PivotPosition {
        AMP(0),
        STOP(-2.809524536132812),
        HOLD(-18.714231491088867),
        TRAP(-74.04991149902344),
        HANDOFF(14.92857551574707),
        SPEAKER(-71.74016571044922);

        public double position;

        private PivotPosition(double position) {
            this.position = position;
        }
    }

    double power = 0.6;

    double anglePower = 0.25;
    double veloSP = .1;

    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor;

    private ArmFeedforward feedForward;
    private SparkMaxPIDController pivotController1;

    private PIDController dumbyController;

    private static RelativeEncoder relativeEncoder;

    private boolean[] connections = new boolean[8];

    private static PivotPosition pivotPosition = PivotPosition.STOP;

    // private static LauncherVoltage launcherVolts = LauncherVoltage.OFF;
    // private static FlickerState flickerState = FlickerState.IN;

    public static Launcher instance;
    public static Intake intake;

    private double reqPower1;

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
        pivotMotor.setOpenLoopRampRate(1);
        pivotMotor.burnFlash();

        pivotController1 = pivotMotor.getPIDController();

        feedForward = new ArmFeedforward(0.01, 0.02, 0.0, 0.0);

        // Prototype numbers
        // upper: .045 lower: .0285 ks:.0085 kg:.037

        relativeEncoder = pivotMotor.getEncoder();

        pivotController1 = pivotMotor.getPIDController();

        pivotController1.setP(LauncherConstants.pivotPCoefficient);
        pivotController1.setI(LauncherConstants.pivotICoefficient);
        pivotController1.setD(LauncherConstants.pivotDCoefficient);

        pivotController1.setFeedbackDevice(relativeEncoder);

        pivotController1.setOutputRange(-0.25, 0.25);

        dumbyController = new PIDController(0.02, 0, 0.0);

        intake = Intake.getInstance();
    }

    public void periodic() {

        pivotMotor.set(dumbyController.calculate(relativeEncoder.getPosition(), pivotPosition.position) + feedForward.calculate(relativeEncoder.getPosition(), veloSP));

        // pivotController1.setReference(pivotPosition.onePosition, ControlType.kPosition, 0,
        //         feedForward.calculate(pivotPosition.onePosition, veloSP));
    }

    public double getReqPower1() {
        return reqPower1;
    }

    public void setLauncherAngle() {
        pivotMotor.set(anglePower + feedForward.calculate(relativeEncoder.getPosition(), veloSP));
    }

    public void setReverseLauncherAngle() {
        pivotMotor.set(-anglePower + feedForward.calculate(relativeEncoder.getPosition(), veloSP));

    }

    public void setAngleStop() {
        pivotMotor.set(0.0);
    }

    public void setLauncherPower() {
        shootMotor1.set(power);
        shootMotor2.set(power);
    }

    public void setReverseLaunch() {
        shootMotor1.set(-power * 0.5);
        shootMotor2.set(-power * 0.5);
    }

    public void setLauncherOff() {
        shootMotor1.set(0.0);
        shootMotor2.set(0.0);
    }

    public void setFlickerOn() {
        flicker.set(.5);
    }

    public void setFlickerReverse() {
        flicker.set(-.5);
    }

    public void setFlickOff() {
        flicker.set(0);
    }

    public void increasePower() {
        power += .1;
    }

    public void decreasePower() {
        power -= .1;
    }

    public double getPosition() {
        return relativeEncoder.getPosition();

    }

    public double getPower() {
        return power;
    }

    public String getPivotPosition() {
        return pivotPosition.toString();
    }

    public  double getLauncherPosition() {
        return relativeEncoder.getPosition();
    }

    public double getPivotVelocitySetPoint() {
        return veloSP;
    }

    public double getPivotCurrent(){
        return pivotMotor.getOutputCurrent();
    }

    public boolean hasReachedPose(double tolerance) {
        if (Math.abs(getLauncherPosition() - pivotPosition.position) > tolerance) {
            return true;
        }
        return false;
    }

    public void setPivotState(PivotPosition state) {
        pivotPosition = state;
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
