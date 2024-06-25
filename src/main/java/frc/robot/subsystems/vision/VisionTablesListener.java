package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.LinkedList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTablesListener {
    private static VisionTablesListener instance;

    private NetworkTableInstance networkTable;
    private NetworkTable visionTable;

    // private StringArraySubscriber tag1Sub;
    private StringArraySubscriber tag3Sub;


    // Transforms of the camera to robot center
    private Transform3d frontCamTransform = new Transform3d(new Translation3d(0.30, -0.26, -0.22),
            new Rotation3d(0, Math.toRadians(33), 0));

    private static LinkedList<AprilTagDetection> frontDetections = new LinkedList<AprilTagDetection>();

    private boolean frontVisible;

    private AprilTagFieldLayout tagLayout;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");
        tag3Sub = visionTable.getStringArrayTopic("Serialized Tags 3").subscribe(new String[] {});

        try {
            tagLayout = AprilTagFieldLayout.loadFromResource("2024-crescendo.json");
        } catch (IOException e) {

        }

    }

    public void printDetects() {
        printLauncherDetects();
        printFrontDetects();
    }

    public void printLauncherDetects() {
        // SmartDashboard.putBoolean("Launcher Tag", getLauncherDetects());
    }
    public void printFrontDetects() {
        SmartDashboard.putBoolean("Front Tag", getFrontDetects());
    }

    public boolean getFrontDetects() {

        if (tag3Sub.get().length > 0)
            frontVisible = true;
        else
            frontVisible = false;

        return frontVisible;
    }

    public Pose2d[] getFrontPoses() {
        String[] serializedTags = tag3Sub.get();
        for (String tagSerial : serializedTags)
            frontDetections.add(new AprilTagDetection(tagSerial));

        if (frontDetections == null || frontDetections.size() == 0)
            return new Pose2d[] {};

        Pose2d[] poses = new Pose2d[frontDetections.size()];
        for (int i = 0; i < poses.length; i++) {
            Pose3d tagFieldPos = tagLayout.getTagPose(frontDetections.get(i).getID()).get();
            Transform3d camTransform = new Transform3d(
                frontCamTransform.getTranslation(),
                new Rotation3d(
                    0,
                    0,
                    frontCamTransform.getRotation().getZ() + frontDetections.get(i).getTransform().getRotation().getZ()
                )
            );
            Transform3d robotPos = frontDetections.get(i).getTransform().plus(camTransform);
            robotPos = new Transform3d(
                    new Translation3d(
                        robotPos.getX() + 0.15,
                        //lab offset: 0.15 comp:.07
                        -robotPos.getY() + .58,
                        //lab:.58
                        -robotPos.getZ()
                    ),
                    robotPos.getRotation()
                );
            poses[i] = tagFieldPos.transformBy(robotPos).toPose2d();

            SmartDashboard.putNumber("VisX", poses[i].getX());
            SmartDashboard.putNumber("VisY", poses[i].getY());

            // Pose3d camFieldPos = tagFieldPos.transformBy(frontDetections.get(i).getTransform());
            // Pose3d robotPos = camFieldPos.transformBy(frontCamTransform);
            // poses[i] = robotPos.toPose2d();
        }
        return poses;
    }

    public double getVisX(){
        return getFrontPoses()[0].getX();
    }

        public double getVisY(){
         return getFrontPoses()[0].getY();
    }

    public double[] getFrontTimeStamps() {
        double[] frontTimeStamps = new double[frontDetections.size()];

        for (int i = 0; i < frontDetections.size(); i++)
            frontTimeStamps[i] = frontDetections.get(i).getTimeStamp();

        return frontTimeStamps;
    }

    public void clearRightDetections(){
         frontDetections.clear();
    }

    public static VisionTablesListener getInstance() {
        if (instance == null)
            instance = new VisionTablesListener();
        return instance;
    }
}
