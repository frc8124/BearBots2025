// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SimDriveTrainSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DefaultSimDriveTrainCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SimDriveTrainSubsystem m_subsystem;
  private DoubleSupplier m_speedSupplier;
  private DoubleSupplier m_rotationSupplier;
  /**
   * Creates a new DefaultSimDriveTrainCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultSimDriveTrainCommand(SimDriveTrainSubsystem subsystem,
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

    m_subsystem.drive(currentSpeed, currentRotation);
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
