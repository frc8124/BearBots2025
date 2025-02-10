// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;

public class DriveSubsystem extends SubsystemBase {
   // 3 meters per second.
   public static final double kMaxSpeed = 3.0;
   // 1/2 rotation per second.
   public static final double kMaxAngularSpeed = Math.PI;
 
   private static final double kTrackWidth = 0.54864;  // meters
   private static final double kWheelRadius = 0.076;  // meters
   private static final double kGearReduction = 10.71;  // ratio
   private static final int kEncoderResolution = 1;   // position count per motor revolution
  
   private final SparkMax m_leftLeader = new SparkMax(DriveTrainConstants.leftLeaderDeviceID, MotorType.kBrushless);
   private final SparkMax m_leftFollower = new SparkMax(DriveTrainConstants.leftFollowerDeviceID, MotorType.kBrushless);
   private final SparkMax m_rightLeader = new SparkMax(DriveTrainConstants.rightLeaderDeviceID, MotorType.kBrushless);
   private final SparkMax m_rightFollower = new SparkMax(DriveTrainConstants.rightFollowerDeviceID, MotorType.kBrushless);

   // An array containing all four motors for doing common operations
   private final SparkMax[] m_motorGroup = { m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower };
   private final SparkMax[] m_leadMotorGroup = { m_leftLeader, m_rightLeader };
 
   private final RelativeEncoder m_leftEncoder;
   private final RelativeEncoder m_rightEncoder;
   
   private final SparkPIDController m_leftPIDController;
   private final SparkPIDController m_rightPIDController;
   
   private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
 
    // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
   
   private final DifferentialDriveKinematics m_kinematics =
       new DifferentialDriveKinematics(kTrackWidth);

   private final DifferentialDriveOdometry m_odometry;
    
    
   // Simulation classes help us simulate our robot
   private final Field2d m_fieldSim = new Field2d();

   private boolean m_bSimulate = false;

    private ADXRS450_GyroSim m_gyroSim;
    private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
            LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3);
    private DifferentialDrivetrainSim m_drivetrainSimulator;


    /** Creates a new DriveSubsystem object */
  public DriveSubsystem() {

    SparkMaxConfig cfgLeftLeader, cfgLeftFollower, cfgRightLeader, cfgRightFollower;

    SparkMaxConfig[] motorConfigs = { cfgLeftLeader, cfgLeftFollower, cfgRightLeader, cfgRightFollower };
    SparkMaxConfig[] leadMotorConfigs = { cfgLeftLeader, cfgRightLeader };

        // Set the conversion factor so encoder getPosition() returns the distamce
    // traveled, given one rotation of wheel covers 2 * Math.PI * kWheelRadius 
    // and the native getPosition() returns number of revolutions of the motor
    // and one revolution of motor is geared down by kGearReduction

    double distPerRev = 2 * Math.PI * kWheelRadius / kEncoderResolution / kGearReduction;
    // set velocity factor to convert RPM to meters per second
    double velFactor = distPerRev / 60d;

    // Set up config objects for all motors.
    for( SparkMaxConfig config : motorConfigs ) {
      config = new SparkMaxConfig();
      config.idleMode(IdleMode.kCoast);
    }

    // Do additional config for the lead motors
    for( SparkMaxConfig config : leadMotorConfigs ) {

      config.closedLoop.pidf(DriveTrainConstants.kP, DriveTrainConstants.kI, DriveTrainConstants.kD, DriveTrainConstants.kFF);
      config.closedLoop.iZone(DriveTrainConstants.kIz);
      config.closedLoop.outputRange(DriveTrainConstants.kMinOutput, DriveTrainConstants.kMaxOutput);

      config.encoder.positionConversionFactor( distPerRev );
      config.encoder.velocityConversionFactor(velFactor);

    }

    // Do some common initialization on all motors
 //   for(SparkMaxConfig config: leadMotorConfigs) {
 //     config.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
 //     m.stopMotor();
 //   }

   // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    cfgRightLeader.inverted(true);

    // Set the configuration for the followers to follow the leaders
    
    cfgLeftFollower.follow(m_leftLeader);
    cfgRightFollower.follow(m_rightLeader);


    // Stop all the motors
    for(SparkMax motor : m_motorGroup) motor.stopMotor();

    
    // Apply the configurations for each motor to the motors themselves
    m_leftLeader.configure(cfgLeftLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightLeader.configure(cfgRightLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftFollower.configure(cfgLeftFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightFollower.configure(cfgRightFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  
    // Set up the encoder objects from the motor controller objects that have
    // already been instantiated in the class level declaraions above
    m_leftEncoder = m_leftLeader.getEncoder();
    m_rightEncoder = m_rightLeader.getEncoder();
 
     // Calculates the next value of the output
     
    m_odometry = new DifferentialDriveOdometry(
      m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    // m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);

    SmartDashboard.putData("Field", m_fieldSim);

  }
  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  /** Sets speeds to the drivetrain motors using the internal PID controller */
  
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {

    //TODO: Fix setting reference in new class objects
//      m_leftPIDController.setReference( speeds.leftMetersPerSecond, SparkMax.ControlType.kVelocity);
//      m_rightPIDController.setReference( speeds.rightMetersPerSecond, SparkMax.ControlType.kVelocity);
      //m_leftLeader.set(speeds.leftMetersPerSecond);
     // m_rightLeader.set(speeds.rightMetersPerSecond);
    }
  
    /**
     * Controls the robot using arcade drive.
     *
     * @param xSpeed the speed for the x axis
     * @param rot the rotation
     */
    public void drive(double xSpeed, double rot) {
        // Full extent joystick will command 1 meter/sec velocity

        rot = rot * Math.PI * 2.0;  // rot of 1 would be 1 rad =  1/2PI deg  = 57 degrees/sec
        SmartDashboard.putNumber("Commanded speed (m/s)", xSpeed);
        SmartDashboard.putNumber("Commanded rotation (rad/s)", rot);

        setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
    }
  
    /** Update robot odometry. */
    public void updateOdometry() {
      m_odometry.update(
          m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

      
    }
  
    /** Resets robot odometry. */
    public void resetOdometry(Pose2d pose) {
      m_leftEncoder.setPosition(0d);
      m_rightEncoder.setPosition(0d);

      if (m_bSimulate)
          m_drivetrainSimulator.setPose(pose);

      m_odometry.resetPosition(
          m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition(), pose);
    }
  
    /** Check the current robot pose. */
    public Pose2d getPose() {
      return m_odometry.getPoseMeters();
    }
  
    /** Update odometry - this should be run every robot loop. */
    public void periodic() {
      updateOdometry();
      m_fieldSim.setRobotPose(m_odometry.getPoseMeters());

      SmartDashboard.putNumber("leftencoder", m_leftEncoder.getPosition());
      SmartDashboard.putNumber("leftvelocity", m_leftEncoder.getVelocity());
    }

    /**
     * Functions to override the hardware objects with simulation classes
     * to allow the subsystem to run within a simulated environment
     *
     */
    public void simulationInit(){
        m_bSimulate = true;

        m_gyroSim = new ADXRS450_GyroSim(m_gyro);
        m_drivetrainSimulator =
        new DifferentialDrivetrainSim(
                m_drivetrainSystem, DCMotor.getNEO(2),  kGearReduction, kTrackWidth, kWheelRadius, null);

//      REVPhysicsSim.getInstance().addSparkMax( m_leftLeader, DCMotor.getNEO(2));
//      REVPhysicsSim.getInstance().addSparkMax( m_rightLeader, DCMotor.getNEO(2));
    }

    public void simulationPeriodic()
    {
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        SmartDashboard.putNumber("Simulate input left volts", m_leftLeader.get());
        SmartDashboard.putNumber("Simulate input right volts", m_rightLeader.get());

        m_drivetrainSimulator.setInputs(
                m_leftLeader.get() * RobotController.getInputVoltage(),
                m_rightLeader.get() * RobotController.getInputVoltage());
        m_drivetrainSimulator.update(0.02);

        SmartDashboard.putNumber("Sim left pos mtrs", m_drivetrainSimulator.getLeftPositionMeters());

        m_leftEncoder.setPosition( m_drivetrainSimulator.getLeftPositionMeters());
        // m_leftEncoder.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
        m_rightEncoder.setPosition(m_drivetrainSimulator.getRightPositionMeters());
        // m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
        m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());}

        public void forwardSlow() {
          //System.out.println("Drivetrain forwardslow");
        
          drive(1,0);
        
        }
        
        public void backwardSlow() {
          //System.out.println("Drivetrain backward slow");
        
          drive(-1,0);
          
          }
        
        public void stop() {
          //System.out.println("Drivetrain stop");
        
        
          drive(0,0);
          
          }
    
    
}
