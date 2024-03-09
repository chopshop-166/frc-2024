package frc.robot.maps.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import com.chopshop166.chopshoplib.maps.SwerveDriveMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class SimCameraSwerveDriveMap extends CameraSwerveDriveMap {

    public VisionSystemSim visionSim = new VisionSystemSim("main");
    public PhotonCameraSim cameraSim;

    public static final Transform3d kDefaultRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0));

    public SimCameraSwerveDriveMap() {
        this(new SwerveDriveMap());
    }

    public SimCameraSwerveDriveMap(final SwerveDriveMap driveMap) {
        super(driveMap, new PhotonCamera("Arducam_OV9782_USB_Camera"), kDefaultRobotToCam);
        SimCameraProperties cameraProp = new SimCameraProperties();
        this.cameraSim = new PhotonCameraSim(camera, cameraProp);
        this.visionSim.addCamera(cameraSim, kDefaultRobotToCam);
    }
}
