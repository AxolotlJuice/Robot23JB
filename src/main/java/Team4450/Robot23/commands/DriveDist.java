package Team4450.Robot23.commands;

import com.revrobotics.CANSparkMax;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.DriveBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class DriveDist extends CommandBase  {
    
    private SynchronousPID          pid1 = new SynchronousPID(0, 0, 0);
    private SynchronousPID          pid2 = new SynchronousPID(0, 0, 0);

    private double                  tolerance1, tolerance2 = .10;

    private DriveBase               driveBase;

    private double                  speed, throttle, strafe, tempTime, elapsedTime, startTime, dist, radians, rotation;
    
    private Pose2d                  startPose, target;

    public DriveDist(DriveBase driveBase, double dist, double radians, double speed){
        this.driveBase = driveBase;
        this.dist = dist;
        this.radians = radians;
        this.speed = speed;
    }

    /* 
    public DriveDist(DriveBase driveBase, double dist, double radians, double speed, double rotation){
        this.driveBase = driveBase;
        this.dist = dist;
        this.radians = radians;
        this.speed = speed;
    }
    */

    public void initialize(){
        //below values are temporary

        startPose = driveBase.getOdometry().getEstimatedPosition();
        //   dist * Math.sin(radians)
        if (speed < 0)
        {
            pid1.setSetpoint(startPose.getX() + dist * Math.cos(radians));
            pid1.setOutputRange(0, -speed);

            pid2.setSetpoint(startPose.getY() + dist * Math.sin(radians));
            pid2.setOutputRange(0, -speed);
        }
        else
        {
            pid1.setSetpoint(startPose.getX() + dist * Math.cos(radians));
            pid1.setOutputRange(0, speed);

            pid2.setSetpoint(startPose.getY() + dist * Math.sin(radians));
            pid2.setOutputRange(0, speed);
        }

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
        return pid1.onTarget(tolerance1) && pid2.onTarget(tolerance2);
    }

    public void end(){
        
    }
}