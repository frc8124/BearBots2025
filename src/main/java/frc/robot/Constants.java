// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveTrainConstants {
    
    public static final int leftLeaderDeviceID = 1; 
    public static final int rightLeaderDeviceID = 4;
    public static final int leftFollowerDeviceID = 2; 
    public static final int rightFollowerDeviceID = 3;

     // PID coefficients
     public static final double kP = 0.5; // 6e-5; 
     public static final double kI = 0;
     public static final double kD = 0; 
     public static final double kIz = 0; 
     public static final double kFF = 0.2; // .000015; 
     public static final double  kMaxOutput = 1; 
     public static final double  kMinOutput = -1;
     public static final double  maxRPM = 5700;  
  }

     public static class IntakeConstants {
      
      public static final double MotorSpeed = 0.4;

     }

}
