package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports;

public class Intake {

    public enum IntakePosition {
        STOP(0.0),
        GROUND(-9.00039100646973),
        TRAP(0.0),
        HANDOFF(-0.999995708465576);

        public double position;

        private IntakePosition(double position) {
            this.position = position;
        }
    }

    private CANSparkMax roller;
    private CANSparkMax flipper;

    public IntakePosition intakePosition = IntakePosition.STOP;
    public static Intake instance;

    private double power = 0.7;

    private double flip = 0.25;

    private ArmFeedforward feedforward;
    private SparkMaxPIDController flipperController;

    private RelativeEncoder encoder;

    private boolean[] connections = new boolean[4];

    private double veloSP = 0.0;

    public Intake() {
        roller = new CANSparkMax(Ports.roller, MotorType.kBrushless);
        roller.restoreFactoryDefaults();

        roller.setSmartCurrentLimit(40);
        roller.setIdleMode(IdleMode.kCoast);
        roller.setInverted(false);
        roller.burnFlash();

        flipper = new CANSparkMax(Ports.flipper, MotorType.kBrushless);
        flipper.restoreFactoryDefaults();

        flipper.setSmartCurrentLimit(70);
        flipper.setIdleMode(IdleMode.kBrake);
        flipper.setInverted(true);

        feedforward = new ArmFeedforward(0.04, 0.05, 0, 0);
        // .037

        // prototype numbers
        // low bound: .022 upper bound:.047 ks: .0125 kg: .0345

        encoder = flipper.getEncoder();

        flipperController = flipper.getPIDController();
        flipperController.setFeedbackDevice(encoder);
        flipperController.setOutputRange(-1.0, 1.0);

        flipperController.setP(IntakeConstants.flipperPCoefficient);
        flipperController.setI(IntakeConstants.flipperICoefficient);
        flipperController.setD(IntakeConstants.flipperDCoefficient);

        flipper.burnFlash();

    }

    public void periodic() {
        flipperController.setReference(intakePosition.position,
        CANSparkMax.ControlType.kPosition, 0,
        feedforward.calculate(encoder.getPosition(), veloSP));
    }

    public void setRollerPower() {
        roller.set(power);
    }

    public void setReverseRollerPower() {
        roller.set(-power);
    }

    public void setFlipperPower() {
        flipper.set(flip + feedforward.calculate(encoder.getPosition(),
                veloSP));
    }

    public void setReverseFlipperPower() {
        flipper.set(-flip + feedforward.calculate(encoder.getPosition(), veloSP));
    }

    public void setFlipperOff() {
        flipper.set(0.0);
    }

    public void setRollerOff() {
        roller.set(0);
    }

    public double getRollerCurrent() {
        return roller.getOutputCurrent();
    }

    public double getFlipperVoltage() {
        return flipper.getBusVoltage();
    }

    public double getFlipperCurrent() {
        return flipper.getOutputCurrent();
    }

    public double getFlipperPosition() {
        return encoder.getPosition();
    }

    public double getFlipperVelocitySetpoint() {
        return veloSP;
    }

    public String getIntakeState() {
        return intakePosition.toString();
    }

    public boolean hasReachedPose(double tolerance) {
        if (Math.abs(encoder.getPosition() - intakePosition.position) > tolerance) {
            return false;
        }
        return true;
    }

    public void setIntakeState(IntakePosition state) {
        intakePosition = state;
    }

    public boolean[] intakeConnections() {
        if (roller.getBusVoltage() != 0) {
            connections[0] = true;
        } else {
            connections[0] = false;
        }

        if (roller.getOutputCurrent() != 0) {
            connections[1] = true;
        } else {
            connections[1] = false;
        }

        if (flipper.getBusVoltage() != 0) {
            connections[2] = true;
        } else {
            connections[2] = false;
        }

        if (flipper.getOutputCurrent() != 0) {
            connections[3] = true;
        } else {
            connections[3] = false;
        }

        return connections;
    }

    public void printConnections() {
        SmartDashboard.putBoolean("roller Voltage", connections[0]);
        SmartDashboard.putBoolean("roller Current", connections[1]);

        SmartDashboard.putBoolean("flipper Voltage", connections[2]);
        SmartDashboard.putBoolean("flipper Current", connections[3]);
    }

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }
}