package frc.robot.subsystems.launcher;

public class LauncherStates {

    public enum LauncherVoltage {
        AMP(0.0),
        SPEAKER(0.0),
        HANDOFF(0.0),
        OFF(0.0);

        public double volts;

        private LauncherVoltage(double volts) {
            this.volts = volts;
        }
    }

    public enum LauncherState {
        AMP(0.0),
        SPEAKER(0.0),
        HANDOFF(0.0),
        RETRACTED(0.0);

        public double position;

        LauncherState(double position) {
            this.position = position;
        }
    }

    public enum FlickerState {
        OUT(0.0),
        IN(0.0);

        public double position;

        private FlickerState(double position) {
            this.position = position;
        }
    }

    public enum LauncherControl {
        MANUAL,
        PID,
    }
}
