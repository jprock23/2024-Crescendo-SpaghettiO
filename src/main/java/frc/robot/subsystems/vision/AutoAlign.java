package frc.robot.subsystems.vision;
import edu.wpi.first.math.controller.PIDController;

public class AutoAlign {
    VisionTablesListener vTListen;

    private double xPose;
    private double yPose;
    private double zRot;

    private double xSP = -.90;
    private double ySP = -.67;
    private double rotSP = 0;

    private PIDController yPoseController = new PIDController(3.0, 0, 0);
    private PIDController xPoseController = new PIDController(3.0, 0, 0);
    private PIDController rotController = new PIDController(1.2, 0, 0);

    public static AutoAlign instance;

    public AutoAlign() {
        vTListen = VisionTablesListener.getInstance();
    }

    public static AutoAlign getInstance() {
        if (instance == null)
            instance = new AutoAlign();
        return instance;
    }
    public double getYSpeed(){
        yPose = vTListen.getY();

        if(!reachYPose(0.05, yPose)) {
            return yPoseController.calculate(yPose, ySP);
        } 
        return 0;
    }

    public double getXSpeed(){
        xPose = vTListen.getX();

        if(!reachYPose(0.05, xPose)) {
            return xPoseController.calculate(xPose, xSP);
        }
        return 0;

    }
    public double getRotSpeed(){
        zRot = vTListen.getRot();

        if(!reachYPose(0.05, zRot)) {
            return rotController. calculate(zRot, rotSP);
        }
        return 0;
    }

    public boolean reachXPose(double tolerance, double measurment){
        if (Math.abs(measurment - xSP) < tolerance){
            return true;
        }
        return false;
    }

    public boolean reachYPose(double tolerance, double measurment){
        if (Math.abs(measurment - ySP) < tolerance){
            return true;
        }
        return false;
    }

    public boolean reachRot(double tolerance, double measurment){
        if (Math.abs(measurment - rotSP) < tolerance){
            return true;
        }
        return false;
    }

    public double getY(){
        return yPose;
    }
}
