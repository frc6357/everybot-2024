package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
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
    // Instantiate the controller
    public DriveTrainSubsystem() {
        super();
        LeftFrontMotor = new CANSparkMax(DrivetrainConstants.kLeftFrontMotorid, MotorType.kBrushless);
        LeftBackMotor = new CANSparkMax(DrivetrainConstants.kLeftRearMotorId, MotorType.kBrushless);
        RightFrontMotor = new CANSparkMax(DrivetrainConstants.kRightFrontMotorId, MotorType.kBrushless);
        RightBackMotor = new CANSparkMax(DrivetrainConstants.kRightRearMotorId, MotorType.kBrushless);

        LeftFrontMotor.setInverted(true);
        LeftBackMotor.setInverted(true);

    }
    // Forward if positive value
    // Backwards is negative value
    // Associate motors with controls
    public void TankDrive(Double LeftSpeed, Double RightSpeed) {
        LeftFrontMotor.set(LeftSpeed);
    }
}
