package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase{
    
    private PhotonCamera camera;


    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public VisionSubsystem() {
        // The camera name is set on the photovision dashboard
        camera = new PhotonCamera("FrontCam");

    
    }

    @Override
    public void periodic() {
        
        PhotonPipelineResult result = camera.getLatestResult();

        boolean hasTargets = result.hasTargets();
        
        if (hasTargets) {
            PhotonTrackedTarget target = result.getBestTarget();
           // get the pitch and yaw of tag
            double yaw = target.getYaw();
            double pitch = target.getPitch();
           
            SmartDashboard.putNumber("Yaw%", yaw);
            SmartDashboard.putNumber("Pitch%", pitch);

            // get the target AprilTag ID
            int aprilTagNumber = target.getFiducialId();
            SmartDashboard.putNumber("ApriTagNo", aprilTagNumber);

        } else {
            SmartDashboard.putNumber("ApriTagNo", 0);

        }

    }
}
