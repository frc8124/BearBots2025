package frc.robot.subsystems;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.OperatorConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;


public class PoseEstimationSubsystem extends SubsystemBase {
    
   // private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

    //private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  //private final Supplier<Rotation2d> rotationSupplier;
  //private final DifferentialDrivePoseEstimator poseEstimator;
  //private final Field2d field2d = new Field2d();
 // private final Notifier photonNotifier = new Notifier(photonEstimator);


 //Camera height in Meters
  final double CAMERA_HEIGHT_METERS = 0.1524;
//Height of target in Meters
    final double TARGET_HEIGHT_METERS = 1.27;
    final double CAMERA_PITCH_RADIANS = 0;
//How far you want to be in Meters
    final double GOAL_RANGE_METERS = 0.6069;

    PhotonCamera camera = new PhotonCamera("FrontCam");

    final double P_GAIN = 0.5;

    final double D_GAIN = 0.0;

    PIDController controller = new PIDController(P_GAIN, 0, D_GAIN);

    CANSparkMax m_leftLeader = new CANSparkMax(DriveTrainConstants.leftLeaderDeviceID, MotorType.kBrushless);
    CANSparkMax m_rightLeader = new CANSparkMax(DriveTrainConstants.rightLeaderDeviceID, MotorType.kBrushless);
    DifferentialDrive drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
}
    