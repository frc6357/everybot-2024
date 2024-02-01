package frc.robot.commands;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.commands.AutoTime;

public class ForwardAuto1{
    DriveTrainSubsystem m_DriveTrain;
    AutoTime AutoMove;
    public void runAuto(){
        AutoMove = new AutoTime(m_DriveTrain, 1 , 1 , 4);
        AutoMove = new AutoTime(m_DriveTrain, .25 , 1 , 2);
    }
}
