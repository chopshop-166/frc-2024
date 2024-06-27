package frc.robot.maps.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.chopshop166.chopshoplib.logging.LogName;
import com.chopshop166.chopshoplib.logging.LoggableMap;
import com.chopshop166.chopshoplib.logging.data.SwerveDriveData;
import com.chopshop166.chopshoplib.maps.SwerveDriveMap;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class CameraSwerveDriveMap implements LoggableMap<CameraSwerveDriveMap.Data> {

    public final SwerveDriveMap driveMap;
    public final PhotonCamera camera;
    public final PhotonPoseEstimator visionEstimator;

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
        @LogName("Pipeline Result")
        public PhotonPipelineResult pipelineResult;
    }
}
