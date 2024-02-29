package frc.robot.subsystems.IO;

import edu.wpi.first.wpilibj.DigitalInput;

public class DigitalInputs {

    public static DigitalInputs instance;

    private DigitalInput port0;
    private DigitalInput port1;
    private DigitalInput port2;
    private DigitalInput port3;
    private DigitalInput port4;
    private DigitalInput port5;
    private DigitalInput port6;
    private DigitalInput port7;
    private DigitalInput port8;
    private DigitalInput port9;

    public DigitalInputs() {

        port0 = new DigitalInput(0);
        port1 = new DigitalInput(1);
        port2 = new DigitalInput(2);
        port3 = new DigitalInput(3);
        port4 = new DigitalInput(4);
        port5 = new DigitalInput(5);
        port6 = new DigitalInput(6);
        port7 = new DigitalInput(7);
        port8 = new DigitalInput(8);
        port9 = new DigitalInput(9);
    }

    public boolean[] getInputs() {
        return new boolean[] {
                port0.get(),
                port1.get(),
                port2.get(),
                port3.get(),
                port4.get(),
                port5.get(),
                port6.get(),
                port7.get(),
                port8.get(),
                port9.get()
        };
    }

    public static DigitalInputs getInstance() {
        if (instance == null) {
            instance = new DigitalInputs();
        }
        return instance;
    }
}
