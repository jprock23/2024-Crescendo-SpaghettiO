package frc.robot.subsystems.launcher;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.Ports;

public class Launcher extends SubsystemBase{

    public enum PivotPosition {
        AMP(22.857, 23.857),
        HANDOFF(1.066666448116302, 1.32857152223587),
        SPEAKER(35.309, 36.309),
        TEST(6.119045257568359, 5.928566455841064);

        public double onePosition;
        public double twoPosition;

        private PivotPosition(double leftPosition, double twoPosition) {
            this.onePosition = leftPosition;
            this.twoPosition = twoPosition;
        }
    }

    double power = 1.0;

    double anglePower = 0.25;
    double veloSP = .1;

    private CANSparkMax shootMotor1;
    private CANSparkMax shootMotor2;

    private CANSparkMax flicker;

    private CANSparkMax pivotMotor1;
    private CANSparkMax pivotMotor2;

    private ArmFeedforward feedForward;
    private SparkMaxPIDController pivotController1;
    private SparkMaxPIDController pivotController2;

    private static RelativeEncoder relativeEncoder1;
    private static RelativeEncoder relativeEncoder2;

    private static PivotPosition pivotPosition = PivotPosition.AMP;

    // private static LauncherVoltage launcherVolts = LauncherVoltage.OFF;
    // private static FlickerState flickerState = FlickerState.IN;

    public static Launcher instance;
    public static Intake intake;

    private double reqPower1;

    public Launcher() {
        shootMotor1 = new CANSparkMax(Ports.shootMotor1, MotorType.kBrushless);
        shootMotor1.restoreFactoryDefaults();

        shootMotor1.setSmartCurrentLimit(60);
        shootMotor1.setIdleMode(IdleMode.kBrake);
        shootMotor1.setInverted(false);
        shootMotor1.burnFlash();

        shootMotor2 = new CANSparkMax(Ports.shootMotor2, MotorType.kBrushless);
        shootMotor2.restoreFactoryDefaults();

        shootMotor2.setSmartCurrentLimit(60);
        shootMotor2.setIdleMode(IdleMode.kCoast);
        shootMotor2.setInverted(true);
        shootMotor2.burnFlash();

        flicker = new CANSparkMax(Ports.flicker, MotorType.kBrushless);
        flicker.restoreFactoryDefaults();

        flicker.setSmartCurrentLimit(20);
        flicker.setIdleMode(IdleMode.kBrake);
        flicker.setInverted(false);
        flicker.burnFlash();

        pivotMotor1 = new CANSparkMax(Ports.pivotMotor1, MotorType.kBrushless);
        pivotMotor1.restoreFactoryDefaults();

        pivotMotor1.setSmartCurrentLimit(60);
        pivotMotor1.setIdleMode(IdleMode.kBrake);
        pivotMotor1.setInverted(false);
        pivotMotor1.setOpenLoopRampRate(1);
        pivotMotor1.burnFlash();

        pivotMotor2 = new CANSparkMax(Ports.pivotMotor2, MotorType.kBrushless);
        pivotMotor2.restoreFactoryDefaults();

        pivotMotor2.setIdleMode(IdleMode.kBrake);
        pivotMotor2.setSmartCurrentLimit(60);
        pivotMotor2.setInverted(true);
        pivotMotor2.setOpenLoopRampRate(1);
        pivotMotor2.burnFlash();

        pivotController1 = pivotMotor1.getPIDController();
        pivotController2 = pivotMotor2.getPIDController();
        feedForward = new ArmFeedforward(0.0085, .037, 0.01, 0.0);
        // upper: .045 lower:

        relativeEncoder1 = pivotMotor1.getEncoder();
        relativeEncoder2 = pivotMotor2.getEncoder();

        pivotController1 = pivotMotor1.getPIDController();
        pivotController2 = pivotMotor2.getPIDController();

        pivotController1.setP(LauncherConstants.pivotPCoefficient);
        pivotController1.setI(LauncherConstants.pivotICoefficient);
        pivotController1.setD(LauncherConstants.pivotDCoefficient);

        pivotController2.setP(LauncherConstants.pivotPCoefficient);
        pivotController2.setI(LauncherConstants.pivotICoefficient);
        pivotController2.setD(LauncherConstants.pivotDCoefficient);

        pivotController1.setFeedbackDevice(relativeEncoder1);
        pivotController2.setFeedbackDevice(relativeEncoder2);

        pivotController1.setOutputRange(-0.6, 0.6);
        pivotController2.setOutputRange(-0.6, 0.6);

        intake = Intake.getInstance();

        // control = new LauncherPID(launchMotor1.getPIDController(),
        // launchMotor2.getPIDController(), launchMotor1.getEncoder(),
        // launchMotor2.getEncoder(),
        // bigFlipper1.getPIDController(), bigFlipper2.getPIDController(),
        // bigFlipper1.getAbsoluteEncoder(Type.kDutyCycle),
        // bigFlipper2.getAbsoluteEncoder(Type.kDutyCycle),
        // flicker.getPIDController(), flicker.getAbsoluteEncoder(Type.kDutyCycle));
    }

    public void periodic() {

        // if(Intake.intakePosition == IntakePosition.HANDOFF){
        // if (intake.getTimeElapsed() > .32 && intake.hasReachedPose(.36)){
        // setReverse();
        // pivotController1.setReference(launcherPosition.onePosition,
        // ControlType.kPosition, 0,
        // pivotFeedforward.calculate(launcherPosition.onePosition, veloSP));
        // pivotController2.setReference(launcherPosition.twoPosition,
        // ControlType.kPosition, 0,
        // pivotFeedforward.calculate(launcherPosition.twoPosition, veloSP));
        // }
        // } else {

        // pivotController1.setReference(pivotPosition.onePosition, ControlType.kPosition, 0,
        //         feedForward.calculate(pivotPosition.onePosition, veloSP));
        // pivotController2.setReference(pivotPosition.twoPosition, ControlType.kPosition, 0,
        //         feedForward.calculate(pivotPosition.twoPosition, veloSP));
    }

    public double getReqPower1() {
        return reqPower1;
    }

    public void setLauncherAngle() {
        pivotMotor1.set(anglePower + feedForward.calculate(relativeEncoder1.getPosition(), veloSP));
        pivotMotor2.set(anglePower - feedForward.calculate(relativeEncoder2.getPosition(), veloSP));
    }

    public void setReverseLauncherAngle() {
        pivotMotor1.set(-anglePower + feedForward.calculate(relativeEncoder1.getPosition(), veloSP));
        pivotMotor2.set(-anglePower + feedForward.calculate(relativeEncoder2.getPosition(), veloSP));

    }

    public void setReverse() {
        shootMotor1.set(-power + feedForward.calculate(relativeEncoder1.getPosition(), veloSP));
        shootMotor2.set(-power + feedForward.calculate(relativeEncoder2.getPosition(), veloSP));
    }

    public void setAngleStop() {
        pivotMotor1.set(0.0);
        pivotMotor2.set(0.0);
    }

    public void setLauncherPower() {
        shootMotor1.set(power);
        shootMotor2.set(power);
    }

    public void setLaunchZero() {
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
        return relativeEncoder1.getPosition();

    }

    public double getPosition2() {
        return relativeEncoder2.getPosition();
    }

    public double getPower() {
        return power;
    }

    public String getPivotPosition() {
        return pivotPosition.toString();
    }

    public static double getLauncherPosition1() {
        // return (turnEncoder1.getPosition() + turnEncoder2.getPosition())/2;
        return relativeEncoder1.getPosition();
    }

    public static double getLauncherPosition2() {
        // return (turnEncoder1.getPosition() + turnEncoder2.getPosition())/2;
        return relativeEncoder2.getPosition();
    }

    public double getPivotVelocitySetPoint() {
        return veloSP;
    }

    public static boolean hasReachedPose(double tolerance) {
        if (Math.abs(getLauncherPosition1() - pivotPosition.onePosition) > tolerance) {
            return true;
        }
        return false;
    }

    // public void setFlickerState(FlickerState state){
    // flickerState = state;
    // }

    public void setPivotState(PivotPosition state) {
        pivotPosition = state;
    }

    // public void setLauncherVolts(LauncherVoltage state){
    // launcherVolts = state;
    // }

    public static Launcher getInstance() {
        if (instance == null)
            instance = new Launcher();
        return instance;
    }
}
