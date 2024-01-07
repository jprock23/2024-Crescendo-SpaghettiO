package frc.robot.subsystems;

import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTablesListener {
    private static VisionTablesListener instance;

    private NetworkTableInstance networkTable;
    private NetworkTable visionTable;
    final IntegerArraySubscriber tagIDSub;
    private IntegerArraySubscriber xCoordsSub;
    private IntegerArraySubscriber yCoordsSub;
    private IntegerArraySubscriber zRotsSub;
    double yPose = 0;
    double xPose = 0;
    double zRot = 0;
    // private IntegerArraySubscriber xEulerSub;
    // private IntegerArraySubscriber yEulerSub;
    // private IntegerArraySubscriber zEulerSub;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");
        tagIDSub = visionTable.getIntegerArrayTopic("IDs").subscribe(new long[] {});
        xCoordsSub = visionTable.getIntegerArrayTopic("X Coords").subscribe(new long[] {});
        yCoordsSub = visionTable.getIntegerArrayTopic("Y Coords").subscribe(new long[] {});
        zRotsSub = visionTable.getIntegerArrayTopic("Z Euler Angles").subscribe(new long[] {});
        // xEulerSub = visionTable.getIntegerArrayTopic("X Euler Angles").subscribe(new
        // long[] {});
        // yEulerSub = visionTable.getIntegerArrayTopic("Y Euler Angles").subscribe(new
        // long[] {});
        // // zEulerSub = visionTable.getIntegerArrayTopic("Z Euler
        // Angles").subscribe(new long[] {});
    }

    public void putInfoOnDashboard() {
        double id = 0;
        boolean tagVisible;

        double[] xPoses;
        double[] yPoses;
        double[] zRots;

        if(tagIDSub.get().length != 0){
            id = convertArray(tagIDSub.get())[0];
            yPoses = convertArray(yCoordsSub.get());
            xPoses = convertArray(xCoordsSub.get());
            zRots = convertArray(zRotsSub.get());
            tagVisible = true;
        } else {
            id = 0;
            xPoses = new double[]{-.90}; 
            yPoses = new double[]{-.67}; 
            zRots = new double[]{.5}; 
            tagVisible = false;
        }
        SmartDashboard.putNumber("IDs", id);
        if(xPoses.length != 0){
            xPose = xPoses[0];
            yPose = yPoses[0];
            zRot = zRots[0];
            
            // SmartDashboard.putNumberArray("IDs", convertArray(tagIDSub.get()));
            SmartDashboard.putNumber("X Coords", xPose);
            SmartDashboard.putNumber("Y Coords", yPose);
            SmartDashboard.putNumber("Z Rot", zRot);
            SmartDashboard.putBoolean("Tag in Sight", tagVisible);
            // SmartDashboard.putNumberArray("X Euler Angles",
            // convertArray(xEulerSub.get()));
            // SmartDashboard.putNumberArray("Y Euler Angles",
            // convertArray(yEulerSub.get()));
            // SmartDashboard.putNumberArray("Z Euler Angles",
            // convertArray(zEulerSub.get()));
    }
    }

    // need to convert each value to double individually, can't typecast entire
    // array
    private double[] convertArray(long[] arr) {
        double[] newArr = new double[arr.length];

        for (int i = 0; i < arr.length; i++)
            newArr[i] = (double) (arr[i]) / 1000.0;

        return newArr;
    }

    public static VisionTablesListener getInstance() {
        if (instance == null)
            instance = new VisionTablesListener();
        return instance;
    }

    public double getRot(){
        return zRot;
    }

    public double getY() {
        return yPose;
    }

      public double getX() {
        return xPose;
    }
}
