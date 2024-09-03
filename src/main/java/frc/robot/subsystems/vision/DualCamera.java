package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.logging.Logger;

/**
 * Singleton class for managing dual cameras and calculating robot pose based on fiducial markers.
 */
public class DualCamera {
    
    
    private static final Logger logger = Logger.getLogger(DualCamera.class.getName());

    private static final PhotonCamera backCamera = new PhotonCamera("BackCam");
    private static final PhotonCamera frontCamera = new PhotonCamera("FrontCam");
;

    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // Define camera offsets in 3D
    private static final Transform3d FRONT_CAMERA_OFFSET = new Transform3d(
        new Translation3d(.325, -.275, 0.24), new Rotation3d(0.0, Math.toRadians(30), Math.toRadians(0.0))
    );
    private static final Transform3d BACK_CAMERA_OFFSET = new Transform3d(
        new Translation3d(-.31, .01, -0.375), new Rotation3d(0.0, Math.toRadians(30), Math.PI)
    );

    private static DualCamera instance;

    private final Map<Integer, Pose3d> fiducialMap = new HashMap<>();

    

    static final PhotonPoseEstimator backPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera,  BACK_CAMERA_OFFSET);
    static final PhotonPoseEstimator frontPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, FRONT_CAMERA_OFFSET);
    private DualCamera() {

        double inchesToMeters = 0.0254;
        

        // Initialize fiducial map with Pose3d
        initializeFiducialMap(inchesToMeters);
    }
    public static boolean hasTargets(PhotonPipelineResult result){
    
        boolean hasTargets = result.hasTargets();
        return hasTargets;
    }
    public static DualCamera getInstance() {
        if (instance == null) {
            instance = new DualCamera();
        }
        return instance;
    }

    public PhotonPipelineResult getBack() {
        return backCamera.getLatestResult();
    }


    public PhotonPipelineResult getFront() {
        return frontCamera.getLatestResult();
    }

    public PhotonPipelineResult getBackCameraResult() {
        return backCamera.getLatestResult();
    }

    public PhotonPipelineResult getFrontCameraResult() {
        return frontCamera.getLatestResult();
    }

    public boolean isBackConnected() {
        return backCamera.isConnected();
    }

    public boolean isFrontConnected() {
        return frontCamera.isConnected();
    }
    
   public Optional<EstimatedRobotPose> getEstimatedPose(Pose2d prevEstimated){
    backPhotonPoseEstimator.setReferencePose(prevEstimated);
    frontPhotonPoseEstimator.setReferencePose(prevEstimated);
    return backPhotonPoseEstimator.update();
   } 

    /**
     * Calculates the robot's position and orientation as Pose2d using 3D camera data.
     *
     * @return Pose2d representing the robot's position and orientation.
     */
    public Pose2d calculateRobotPosition() {
        Pose3d frontRobotPose = calculatePoseFromCameraResult(getFrontCameraResult(), FRONT_CAMERA_OFFSET);
        Pose3d backRobotPose = calculatePoseFromCameraResult(getBackCameraResult(), BACK_CAMERA_OFFSET);

        // if ( backRobotPose != null) {
        //     // Average X, Y, and Z from both camera results
        //     double avgX = (frontRobotPose.getX() + backRobotPose.getX()) / 2;
        //     double avgY = (frontRobotPose.getY() + backRobotPose.getY()) / 2;
        //     // We can ignore Z for Pose2d
        //     Rotation3d avgRotation = new Rotation3d(
        //         (frontRobotPose.getRotation().getX() + backRobotPose.getRotation().getX()) / 2,
        //         (frontRobotPose.getRotation().getY() + backRobotPose.getRotation().getY()) / 2,
        //         (frontRobotPose.getRotation().getZ() + backRobotPose.getRotation().getZ()) / 2
        //     );

        //     // Convert the 3D Pose to 2D Pose
        //     return new Pose2d(avgX, avgY, new Rotation2d(avgRotation.getZ()));
        // }

        // Return the Pose2d with available data, or a default Pose2d if none is available
        return frontRobotPose != null ? new Pose2d(frontRobotPose.getX(), frontRobotPose.getY(), new Rotation2d(frontRobotPose.getRotation().getZ())) :
               backRobotPose != null ? new Pose2d(backRobotPose.getX(), backRobotPose.getY(), new Rotation2d(backRobotPose.getRotation().getZ())) :
               new Pose2d();
    }

    private Pose3d calculatePoseFromCameraResult(PhotonPipelineResult result, Transform3d cameraOffset) {
        if (result != null && result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            // Optional<Pose3d> aprilPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
            // Transform3d transformPhoton = target.getBestCameraToTarget();
            // Pose3d robotToTag = PhotonUtils.estimateFieldToRobotAprilTag(transformPhoton, aprilPose.get(), cameraOffset);
            // return robotToTag;
                
            
            Pose3d fiducialPose = fiducialMap.get(target.getFiducialId());

            if (fiducialPose != null) {
                Transform3d transform = target.getBestCameraToTarget().inverse();
                Pose3d cameraToTargetPose = fiducialPose.transformBy(transform);

                Pose3d robotPose3d = cameraToTargetPose.transformBy(cameraOffset);
                return new Pose3d(
                    robotPose3d.getX(),
                    robotPose3d.getY(),
                    robotPose3d.getZ(),
                    robotPose3d.getRotation()
                );
            }
        }
        return null;
    }

    private void initializeFiducialMap(double inchesToMeters) {
        fiducialMap.put(1, new Pose3d(593.68 * inchesToMeters, 9.68 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(120))));
        fiducialMap.put(2, new Pose3d(637.21 * inchesToMeters, 34.79 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(120))));
        fiducialMap.put(3, new Pose3d(652.73 * inchesToMeters, 196.17 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(180))));
        fiducialMap.put(4, new Pose3d(652.73 * inchesToMeters, 218.42 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(180))));
        fiducialMap.put(5, new Pose3d(578.77 * inchesToMeters, 323.00 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(270))));
        fiducialMap.put(6, new Pose3d(72.50 * inchesToMeters, 323.00 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(270))));
        fiducialMap.put(7, new Pose3d(-1.50 * inchesToMeters, 218.42 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(0))));
        fiducialMap.put(8, new Pose3d(-1.50 * inchesToMeters, 196.17 * inchesToMeters, 57.13 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(0))));
        fiducialMap.put(9, new Pose3d(14.02 * inchesToMeters, 34.79 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(60))));
        fiducialMap.put(10, new Pose3d(57.54 * inchesToMeters, 9.68 * inchesToMeters, 53.38 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(60))));
        fiducialMap.put(11, new Pose3d(468.69 * inchesToMeters, 146.19 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(300))));
        fiducialMap.put(12, new Pose3d(468.69 * inchesToMeters, 177.10 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(60))));
        fiducialMap.put(13, new Pose3d(441.74 * inchesToMeters, 161.62 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(180))));
        fiducialMap.put(14, new Pose3d(209.48 * inchesToMeters, 161.62 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(0))));
        fiducialMap.put(15, new Pose3d(182.73 * inchesToMeters, 177.10 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(120))));
        fiducialMap.put(16, new Pose3d(182.73 * inchesToMeters, 146.19 * inchesToMeters, 52.00 * inchesToMeters, new Rotation3d(0.0, 0.0, Math.toRadians(240))));
    }
}