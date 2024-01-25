package frc.robot.commands;


import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TankDrive;
import edu. wpi.first.wpilibj.Timer;

public class AutoTime extends TankDrive {

  private Timer _timer;
  private double end_time;
  public AutoTime(DriveTrainSubsystem m_DriveTrain , double m_leftspeed ,double m_rightspeed, double end_time) {
    super(m_DriveTrain, ()->{return m_leftspeed;}, ()->{return m_rightspeed;});
    _timer = new Timer();
    this.end_time=end_time;
  }
  @Override
  public void initialize(){
    _timer.start();
  }

  @Override
  public boolean isFinished(){
      return _timer.get() >= end_time;
  }
  
     
}