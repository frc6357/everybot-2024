package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DriveTrainSubsystem extends SubsystemBase{
    // velocity factor is determined by (1/60)*(1/GearReduction)* pi(wheelDiameter)
    private static final double ENCODER_RPM_TO_MPS = 0.0009433714051;
    //position factor is determined by (1/GearReduction)* pi(wheelDiameter) or for now velocity factor*60
    private static final double ENCODER_POS_TO_METERS = ENCODER_RPM_TO_MPS*60;
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
    

    Pigeon2 pigeon;
    private RelativeEncoder rightFrontEncoder;
    private RelativeEncoder leftFrontEncoder;
    private DifferentialDriveOdometry odometry;
    private SparkPIDController leftPidController;
    private SparkPIDController rightPidController;
    private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.0));

    // Instantiate the controller
    public DriveTrainSubsystem() {
        super();
        pigeon=new Pigeon2(10);
        LeftFrontMotor = new CANSparkMax(DrivetrainConstants.kLeftFrontMotorid, MotorType.kBrushless);
        LeftBackMotor = new CANSparkMax(DrivetrainConstants.kLeftRearMotorId, MotorType.kBrushless);
        RightFrontMotor = new CANSparkMax(DrivetrainConstants.kRightFrontMotorId, MotorType.kBrushless);
        RightBackMotor = new CANSparkMax(DrivetrainConstants.kRightRearMotorId, MotorType.kBrushless);

        //Encoder setup
        leftFrontEncoder = LeftFrontMotor.getEncoder();
        rightFrontEncoder = RightFrontMotor.getEncoder();
        leftFrontEncoder.setPositionConversionFactor(ENCODER_POS_TO_METERS);
        leftFrontEncoder.setVelocityConversionFactor(ENCODER_RPM_TO_MPS);
        rightFrontEncoder.setPositionConversionFactor(ENCODER_POS_TO_METERS);
        rightFrontEncoder.setVelocityConversionFactor(ENCODER_RPM_TO_MPS);

        leftFrontEncoder.getPosition();
        LeftFrontMotor.setInverted(true);
        LeftBackMotor.follow(LeftFrontMotor);

        RightBackMotor.follow(RightFrontMotor);
        differentialDrive = new DifferentialDrive(LeftFrontMotor,RightFrontMotor);
        
        //Pid setup
        leftPidController = LeftFrontMotor.getPIDController();
        rightPidController = RightFrontMotor.getPIDController();
        double kP =6e-5;
        leftPidController.setP(kP);
        rightPidController.setP(kP);
        leftPidController.setOutputRange(-1, 1);
        rightPidController.setOutputRange(-1, 1);

        //odometry
        odometry = new DifferentialDriveOdometry(pigeon.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());



    }
    
    // Forward if positive value
    // Backwards is negative value
    // Associate motors with controls
    public void TankDrive(Double LeftSpeed, Double RightSpeed) {

        differentialDrive.tankDrive(LeftSpeed,RightSpeed);
    }
    public double getHeading(){
        return pigeon.getYaw().getValueAsDouble();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Heading", getHeading());
        odometry.update(pigeon.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition());
        SmartDashboard.putNumber("LeftVelocity", leftFrontEncoder.getVelocity());
        SmartDashboard.putNumber("RightVelocity", rightFrontEncoder.getVelocity());
    }
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(pigeon.getRotation2d(), leftFrontEncoder.getPosition(), rightFrontEncoder.getPosition(),pose);
    }
    public ChassisSpeeds getWheelSpeeds(){
        DifferentialDriveWheelSpeeds wheelSpeeds =  new DifferentialDriveWheelSpeeds(leftFrontEncoder.getVelocity(),rightFrontEncoder.getVelocity());
        return kinematics.toChassisSpeeds(wheelSpeeds);
    }
    public void driveVelocity(ChassisSpeeds speed){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speed);
        SmartDashboard.putNumber("LeftVelocity", wheelSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("RightVelocity", wheelSpeeds.rightMetersPerSecond);
        leftPidController.setReference(wheelSpeeds.leftMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        rightPidController.setReference(wheelSpeeds.rightMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    }
    public void SetPathPlanner(){
        AutoBuilder.configureRamsete(
            this::getPose,
            this::resetPose,
            this::getWheelSpeeds,
            this::driveVelocity,
            new ReplanningConfig(),
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;

                }
                return false;
            },
            this
        );
    }

}
