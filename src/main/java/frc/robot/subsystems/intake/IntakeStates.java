package frc.robot.subsystems.intake;

public class IntakeStates {
    
    public enum IntakePosition{
        GROUND(0.0),
        HANDOFF(0.0),
        RETRACTED(0.0);

        public double position;

        private IntakePosition(double position){
            this.position = position;
        }
    }
}
