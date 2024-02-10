// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class VelocityDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubsystem m_DriveTrain;

  public final Supplier<Double> m_LeftSpeedSupplier, m_RightSpeedSupplier; 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VelocityDrive(DriveTrainSubsystem subsystem, Supplier<Double> LeftSpeed, Supplier<Double> RightSpeed) {
    m_DriveTrain = subsystem;
    m_LeftSpeedSupplier = LeftSpeed;
    m_RightSpeedSupplier = RightSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrain.driveVelocity(new ChassisSpeeds(m_LeftSpeedSupplier.get(), m_RightSpeedSupplier.get(), 0.0));
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
