// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// add SUBSYSTEMFILE to constances file
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrainSubsystem;
import java.io.File;
import java.io.IOException;
import java.util.Optional;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.utils.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private Optional<DriveTrainSubsystem>  m_DriveTrain  = Optional.empty();
  private Optional<LimeLightSubsystem>  m_LimeLightSubsystem  = Optional.empty();

  private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureSubsystems();
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureSubsystems()
    {
        File deployDirectory = Filesystem.getDeployDirectory();

        ObjectMapper mapper = new ObjectMapper();
        JsonFactory factory = new JsonFactory();

        try
        {
            // Looking for the Subsystems.json file in the deploy directory
            JsonParser parser =
                    factory.createParser(new File(deployDirectory, Constants.SUBSYSTEMFILE));
            
            SubsystemControls subsystems = mapper.readValue(parser, SubsystemControls.class);

            // Instantiating subsystems if they are present
            // This is decided by looking at Subsystems.json
            if (subsystems.islimelightPresent())
            {
              m_LimeLightSubsystem = Optional.of(new LimeLightSubsystem());
            }

            if(subsystems.isdrivePresent())
            {
              m_DriveTrain = Optional.of(new DriveTrainSubsystem());
            }
            
            
    

            //     Configures the autonomous paths and smartdashboard chooser
            //     new SK23AutoGenerator(driveSubsystem.get(), armSubsystem, intakeSubsystem);
            //     autoCommandSelector = AutoBuilder.buildAutoChooser();
            //     SmartDashboard.putData("Auto Chooser", autoCommandSelector);
            
            
        }
        catch (IOException e)
        {
            DriverStation.reportError("Failure to read Subsystem Control File!", e.getStackTrace());
        }
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*new Trigger(m_Drivet::exampleCondition)
        .onTrue(new TankDrive(m_exampleSubsystem));*/

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    if(m_LimeLightSubsystem.isPresent()){
      LimeLightSubsystem limelight = m_LimeLightSubsystem.get();
      m_driverController.a().onTrue(new InstantCommand(() -> limelight.GetRobotPosition()));
    }

    if(m_DriveTrain.isPresent())
    {
      DriveTrainSubsystem drive = m_DriveTrain.get();
      drive.setDefaultCommand(
        new TankDrive(drive, 
        () -> m_driverController.getLeftY(), 
        () -> m_driverController.getRightY()));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // An example command will be run in autonomous
    return autoChooser.getSelected();

  }
}
