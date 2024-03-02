package frc.robot.subsystems.launcher;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.IO.DigitalInputs;
import frc.robot.Ports;

public class Launcher {

    public enum LauncherState {
        AMP(-65, 0.225),
        START(-4.809524536132812, 0.0),
        TRAP(-70.04991149902344, 0.8),
        LONG(-10, 1.0),
        HANDOFF(8.92857551574707, 0.35),
        SPEAKER(-55.0, 1.0);

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

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor;

    private ArmFeedforward feedForward;
    private SparkMaxPIDController pivotController1;

    private TrapezoidProfile motionProfile;
    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setPoint = new TrapezoidProfile.State(LauncherState.START.position, 0);

    private double veloSP = 4000;
    private double maxAccel = 3000;

    private static RelativeEncoder encoder;
    private DigitalInputs breakBeam;

    private boolean[] connections = new boolean[8];

    private static LauncherState launchState = LauncherState.START;

    public static Launcher instance;

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

        feedForward = new ArmFeedforward(0.0, 0.4

                , 0.1, 0.0);
        motionProfile = new TrapezoidProfile(new Constraints(veloSP, maxAccel));

        // Prototype numbers
        // upper: .045 lower: .0285 ks:.0085 kg:.037

        encoder = pivotMotor.getEncoder();

        pivotController1 = pivotMotor.getPIDController();

        pivotController1.setP(LauncherConstants.pivotPCoefficient);
        pivotController1.setI(LauncherConstants.pivotICoefficient);
        pivotController1.setD(LauncherConstants.pivotDCoefficient);

        pivotController1.setFeedbackDevice(encoder);

        pivotController1.setOutputRange(-1, 1);

        pivotMotor.burnFlash();

        breakBeam = DigitalInputs.getInstance();

    }

    public void updatePose() {

        setPoint = motionProfile.calculate(0.02, setPoint, goal);

        pivotController1.setReference(launchState.position, CANSparkMax.ControlType.kPosition, 0,
                feedForward.calculate(encoder.getPosition(), 0));

    }

    public void setPivotPower() {
        pivotMotor.set(anglePower + feedForward.calculate(encoder.getPosition(), veloSP));
    }

    public void setReversePivotPower() {
        pivotMotor.set(-anglePower + feedForward.calculate(encoder.getPosition(), veloSP));

    }

    public void setPivotOff() {
        pivotMotor.set(0.0);
    }

    public void setLauncherOn() {
        shootMotor1.set(launchState.launchSpeed);
        shootMotor2.set(launchState.launchSpeed);
    }

    public void setReverseLauncherOn() {
        shootMotor1.set(-launchState.launchSpeed);
        shootMotor2.set(-launchState.launchSpeed);
    }

    public void setLauncherOff() {
        shootMotor1.set(0.0);
        shootMotor2.set(0.0);
    }

    public void setFlickerOn() {
        flicker.set(0.8);
    }

    public void setFlickerReverse() {
        flicker.set(-0.8);
    }

    public void setFlickOff() {
        flicker.set(0);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean getBreakBeam() {
        return !breakBeam.getInputs()[Ports.breakBeam];
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
        goal = new TrapezoidProfile.State(launchState.position, 0);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public double getVelocityGoal() {
        return setPoint.velocity;
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
