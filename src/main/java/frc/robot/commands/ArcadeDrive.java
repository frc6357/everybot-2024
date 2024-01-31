// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
  public class ArcadeDrive extends Command {
    public Supplier<Double> leftSpeed;
    public Supplier<Double> rightSpeed;
  public DriveTrainSubsystem m_DriveTrain;
    //public void teleopPeriodic() {
      // Drive with arcade drive.
      // That means that the Y axis drives forward
      // and backward, and the X turns left and right.
    // m_robotDrive.ArcadeDrive(m_driverController.getRightX(), m_driverController.getLeftY());
    // }
    public ArcadeDrive(DriveTrainSubsystem subsystem, Supplier<Double> LeftSpeed, Supplier<Double> RightSpeed) {
      m_DriveTrain = subsystem;
      leftSpeed = LeftSpeed;
      rightSpeed = RightSpeed;
      //m_LeftSpeedSupplier = LeftSpeed;
      //m_RightSpeedSupplier = RightSpeed;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      // Drive with arcade drive.
      // That means that the Y axis drives forward
      // and backward, and the X turns left and right.
      m_DriveTrain.ArcadeDrive(leftSpeed.get(), rightSpeed.get());
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




