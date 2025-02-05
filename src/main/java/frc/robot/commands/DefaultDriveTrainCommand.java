// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DefaultDriveTrainCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  private DoubleSupplier m_speedSupplier;
  private DoubleSupplier m_rotationSupplier;
  SlewRateLimiter filter = new SlewRateLimiter(5);

  //please let rachel connery crnkovich touch code uwu

  /**
   * Creates a new DefaultSimDriveTrainCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultDriveTrainCommand(DriveSubsystem subsystem,
      DoubleSupplier speedSupplier,
      DoubleSupplier rotationSupplier
  ) {
    m_subsystem = subsystem;
    m_speedSupplier = speedSupplier;
    m_rotationSupplier = rotationSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentSpeed = m_speedSupplier.getAsDouble();
    double currentRotation = m_rotationSupplier.getAsDouble();
    
    double limitedSpeed = filter.calculate(currentSpeed);

    // currently the speeds are given in meters/sec..
    m_subsystem.drive(limitedSpeed, currentRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
