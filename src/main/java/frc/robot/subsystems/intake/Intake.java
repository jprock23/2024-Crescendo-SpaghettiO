package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Ports;

public class Intake extends SubsystemBase{

    public enum IntakePosition {
        GROUND(-9.35),
        HANDOFF(-1.0);

        public double position;

        private IntakePosition(double position) {
            this.position = position;
        }
    }

    private CANSparkMax roller;
    private CANSparkMax flipper;

    // private IntakePID control;

    public IntakePosition intakePosition = IntakePosition.GROUND;
    public static Intake instance;

    private double power = .4;

    private double flip = 0.25;

    private ArmFeedforward feedforward;
    private SparkMaxPIDController flipperController;

    private RelativeEncoder relativeEncoder;

    private double veloSP = .02;
    private double startTime = 0.0;
    private double timeElapsed = 0.0;

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
        flipper.burnFlash();

        feedforward = new ArmFeedforward(0.0125, 0.0345, 0, 0);
        // low bound: .022 upper bound:.047

        relativeEncoder = flipper.getEncoder();

        flipperController = flipper.getPIDController();
        flipperController.setFeedbackDevice(relativeEncoder);
        flipperController.setOutputRange(-0.25, 0.25);

        flipperController.setP(IntakeConstants.flipperPCoefficient);
        flipperController.setI(IntakeConstants.flipperICoefficient);
        flipperController.setD(IntakeConstants.flipperDCoefficient);

        // control = IntakePID.getInstance(flipper.getPIDController(),
        // flipper.getAbsoluteEncoder(Type.kDutyCycle));
    }

    public void periodic() {

        flipperController.setReference(intakePosition.position, ControlType.kPosition, 0,
                feedforward.calculate(relativeEncoder.getPosition(), veloSP));

        // if (intakePosition == IntakePosition.HANDOFF && hasReachedPose(.36)) {

        // // if (Launcher.hasReachedPose(.55)) {

        // if (startTime == 0.0) {
        // startTime = Timer.getFPGATimestamp();
        // }

        // timeElapsed = Timer.getFPGATimestamp() - startTime;
        // SmartDashboard.putNumber("Time Elpased", timeElapsed);

        // if (timeElapsed < .32 && timeElapsed != Timer.getFPGATimestamp()) {
        // setRollerPower();
        // } else if (timeElapsed > .32 && timeElapsed < 2 && timeElapsed !=
        // Timer.getFPGATimestamp()) {
        // setRollerOff();
        // } else if ( timeElapsed < 3 && timeElapsed > 2 && timeElapsed !=
        // Timer.getFPGATimestamp()) {
        // setReverseRollerPower();
        // } else if (timeElapsed > 3 && timeElapsed != Timer.getFPGATimestamp()) {
        // setRollerOff();
        // }
        // }
        // .31 works

    }

    public double getStartTime() {
        return startTime;
    }

    public double getTimeElapsed() {
        return timeElapsed;
    }

    public void resetStartTime() {
        startTime = 0.0;
    }

    public void reverseFlipper() {
        flipper.set(-flip + feedforward.calculate(relativeEncoder.getPosition(), veloSP));
    }

    public void setRollerPower() {
        roller.set(power);
    }

    public void setReverseRollerPower() {
        roller.set(-power);
    }

    public void setFlipperPower() {
        flipper.set(flip + feedforward.calculate(relativeEncoder.getPosition(), veloSP));
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

    public double getFlipperPosition() {
        return relativeEncoder.getPosition();
    }

    public double getFlipperVelocitySetpoint() {
        return veloSP;
    }

    public String getIntakeState() {
        return intakePosition.toString();
    }

    public boolean hasReachedPose(double tolerance) {
        if (Math.abs(relativeEncoder.getPosition() - intakePosition.position) > tolerance) {
            return false;
        }
        return true;
    }

    public void setIntakeState(IntakePosition state) {
        intakePosition = state;
    }

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }
}