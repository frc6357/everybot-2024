package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DriveTrainSubsystem extends SubsystemBase{
    
    // Four motors
    // Instantiate LeftFrontMotor
    CANSparkMax LeftFrontMotor; 
    // Instantiate LeftBackMotor
    CANSparkMax LeftBackMotor;
    // Instantiate RightFrontMotor
    CANSparkMax RightFrontMotor;
    // Instantiate RightBackMotor
    CANSparkMax RightBackMotor;
    // Instantiate differentialDrive
    DifferentialDrive differentialDrive;
    
    // Instantiate the controller
    public DriveTrainSubsystem() {
        super();
        LeftFrontMotor = new CANSparkMax(DrivetrainConstants.kLeftFrontMotorid, MotorType.kBrushless);
        LeftBackMotor = new CANSparkMax(DrivetrainConstants.kLeftRearMotorId, MotorType.kBrushless);
        RightFrontMotor = new CANSparkMax(DrivetrainConstants.kRightFrontMotorId, MotorType.kBrushless);
        RightBackMotor = new CANSparkMax(DrivetrainConstants.kRightRearMotorId, MotorType.kBrushless);

        LeftFrontMotor.setInverted(true);
        LeftBackMotor.follow(LeftFrontMotor);

        RightBackMotor.follow(RightFrontMotor);
 
        differentialDrive = new DifferentialDrive(LeftFrontMotor,RightFrontMotor);
    }
    // Forward if positive value
    // Backwards is negative value
    // Associate motors with controls
    
    public void ArcadeDrive(Double xSpeed, Double RotationSpeed) {
        differentialDrive.arcadeDrive(xSpeed,RotationSpeed);
    }
    public void TankDrive(Double LeftSpeed, Double RightSpeed) {
        differentialDrive.tankDrive(LeftSpeed,RightSpeed);
    }
}
