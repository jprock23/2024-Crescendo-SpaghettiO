package frc.robot.subsystems.vision;

import java.io.IOException;
import java.util.LinkedList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArraySubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionTablesListener {
    private static VisionTablesListener instance;

    private NetworkTableInstance networkTable;
    private NetworkTable visionTable;

    private StringArraySubscriber tag1Sub;
    private StringArraySubscriber tag3Sub;


    // Transforms of the camera to robot center
    private Transform3d launcherCamTransform = new Transform3d(new Translation3d(-0.3, 0, -0.37),
            new Rotation3d(0, 0, Units.degreesToRadians(-179)));
            //Math.toRadians(146.5)

            // private Transform3d frontCamTransform = new Transform3d(new Translation3d(0.3302, -0.2762504, 0.2306066),
            // new Rotation3d(0, Math.toRadians(-35), Math.toRadians(-90)));
    private Transform3d frontCamTransform = new Transform3d(new Translation3d(0.30, -0.26, -0.22),
        // private Transform3d frontCamTransform = new Transform3d(new Translation3d(0.315, -0.27, -0.23),
            new Rotation3d(0, Math.toRadians(33), 0));

    private static LinkedList<AprilTagDetection> launcherDetections = new LinkedList<AprilTagDetection>();
    private static LinkedList<AprilTagDetection> frontDetections = new LinkedList<AprilTagDetection>();

    private boolean launcherVisible;
    private boolean frontVisible;

    private AprilTagFieldLayout tagLayout;

    public VisionTablesListener() {
        networkTable = NetworkTableInstance.getDefault();
        visionTable = networkTable.getTable("Vision");
        tag1Sub = visionTable.getStringArrayTopic("Serialized Tags").subscribe(new String[] {});
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
        SmartDashboard.putBoolean("Launcher Tag", getLauncherDetects());
    }
    public void printFrontDetects() {
        SmartDashboard.putBoolean("Front Tag", getFrontDetects());
    }

    public boolean getLauncherDetects() {
        if (tag1Sub.get().length > 0)
            launcherVisible = true;
        else
            launcherVisible = false;

        return launcherVisible;
    }

    public boolean getFrontDetects() {

        if (tag3Sub.get().length > 0)
            frontVisible = true;
        else
            frontVisible = false;

        return frontVisible;
    }

    public Pose2d[] getLauncherPoses() {
        String[] serializedTags = tag1Sub.get();
        for (String tagSerial : serializedTags)
            launcherDetections.add(new AprilTagDetection(tagSerial));

        if (launcherDetections == null || launcherDetections.size() == 0)
            return new Pose2d[] {};

        Pose2d[] poses = new Pose2d[launcherDetections.size()];
        for (int i = 0; i < poses.length; i++) {
            Pose3d tagFieldPos = tagLayout.getTagPose(launcherDetections.get(i).getID()).get();
            Transform3d robotPos = launcherDetections.get(i).getTransform().plus(launcherCamTransform);
            robotPos = new Transform3d(
                    new Translation3d(
                        robotPos.getX(),
                        -robotPos.getY(),
                        -robotPos.getZ()
                    ),
                    robotPos.getRotation()
                );
            poses[i] = tagFieldPos.transformBy(robotPos).toPose2d();

            // Pose3d camFieldPos = tagFieldPos.transformBy(launcherDetections.get(i).getTransform());
            // Pose3d robotPos = camFieldPos.transformBy(launcherCamTransform);
            //poses[i] = robotPos.toPose2d();
        }

        return poses;
    }

    public double[] getLauncherTimeStamps() {
        double[] launcherTimestamps = new double[launcherDetections.size()];

        for (int i = 0; i < launcherDetections.size(); i++)
            launcherTimestamps[i] = launcherDetections.get(i).getTimeStamp();

        return launcherTimestamps;
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
                        robotPos.getX() + .15,
                        -robotPos.getY() + .58,
                        -robotPos.getZ()
                    ),
                    robotPos.getRotation()
                );
            poses[i] = tagFieldPos.transformBy(robotPos).toPose2d();

            SmartDashboard.putNumber("Vis    b   X", poses[i].getX());
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

    public void clearLauncherDetections(){
         launcherDetections.clear();
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
