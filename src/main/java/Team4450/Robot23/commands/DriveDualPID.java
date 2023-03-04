package Team4450.Robot23.commands;

import com.revrobotics.CANSparkMax;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class DriveDualPID extends CommandBase  {
    
    private SynchronousPID          pid1 = new SynchronousPID(0, 0, 0);
    private SynchronousPID          pid2 = new SynchronousPID(0, 0, 0);

    private DriveBase               driveBase;

    private double                  speed, throttle, strafe, tempTime, elapsedTime, startTime;
    
    private Pose2d                  target;

    public DriveDualPID(DriveBase driveBase, Pose2d target, double speed){
        this.driveBase = driveBase;
        this.target = target;
        this.speed = speed;
    }

    public void initialize(){
        //below values are temporary
        if (speed < 0)
			{
				pid1.setSetpoint(-target.getX());
				pid1.setOutputRange(speed, 0);

                pid1.setSetpoint(-target.getX());
				pid1.setOutputRange(speed, 0);
			}
			else
			{
				pid1.setSetpoint(target.getY());
				pid1.setOutputRange(0, speed);

                pid1.setSetpoint(target.getY());
				pid1.setOutputRange(0, speed);
			}

        pid1.setSetpoint(target.getX());
        pid2.setSetpoint(target.getY());

        startTime = Util.timeStamp();
        tempTime = Util.timeStamp();

    }
    
    public void execute(boolean intrrupted){
        
        elapsedTime = Util.getElaspedTime(tempTime);

        throttle = pid1.calculate(driveBase.getOdometry().getEstimatedPosition().getX(), tempTime);
        strafe = pid2.calculate(driveBase.getOdometry().getEstimatedPosition().getY(), tempTime);

        driveBase.drive(throttle, strafe, 0.0);
    }

    public boolean isFinished(){
        return (armMotor1.getEncoder().getVelocity() == 0.0) && (armMotor2.getEncoder().getVelocity() == 0.0);
    }

    public void end(){
        
    }
}
