package frc.robot.maps.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.PhotonPipelineResult;

import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.SwerveDriveData;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class CameraSwerveDriveMap implements LoggableMap<CameraSwerveDriveMap.Data> {

    public final SwerveDriveMap driveMap;
    public final PhotonCamera camera;
    public final PhotonPoseEstimator visionEstimator;
    public Optional<PhotonCameraSim> cameraSim = Optional.empty();

    public static final Transform3d kDefaultRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0));

    public CameraSwerveDriveMap() {
        this(new SwerveDriveMap());
    }

    public CameraSwerveDriveMap(final SwerveDriveMap driveMap) {
        this(driveMap, new PhotonCamera("Arducam_OV9782_USB_Camera"), kDefaultRobotToCam);
        SimCameraProperties cameraProp = new SimCameraProperties();
        this.cameraSim = Optional.of(new PhotonCameraSim(camera, cameraProp));
    }

    public CameraSwerveDriveMap(final SwerveDriveMap driveMap, final PhotonCamera camera,
            final Transform3d robotToCam) {
        this.driveMap = driveMap;
        this.camera = camera;
        this.visionEstimator = new PhotonPoseEstimator(
                AprilTagFields.kDefaultField.loadAprilTagLayoutField(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
        this.visionEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void updateData(Data data) {
        driveMap.updateData(data);
        data.pipelineResult = camera.getLatestResult();
    }

    public static class Data extends SwerveDriveData {
        public PhotonPipelineResult pipelineResult;

        @Override
        public void toLog(LogTable table) {
            super.toLog(table);
            table.put("Pipeline Result", pipelineResult);
        }

        @Override
        public void fromLog(LogTable table) {
            super.fromLog(table);
            pipelineResult = table.get("Pipeline Result", pipelineResult);
        }
    }
}
