package Team4450.Robot23.commands;

import com.revrobotics.CANSparkMax;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.JWinch;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WinchToTarget extends CommandBase  {
    
    private SynchronousPID          pidCon = new SynchronousPID(0, 0, 0);

    private JWinch                   winch;

    private CANSparkMax             motor;
    
    private double                  lastTimeCalled, startTime, elapsedTime;
    private double                  targetPosition, power;
    private double                  tolerance = .5;

    public WinchToTarget(JWinch winch, int targetPosition){
        this.winch = winch;
        this.targetPosition = targetPosition;
        
    }

    public void initialize(){
        //below values are temporary
        pidCon.setOutputRange(0.5, 0.0);

        pidCon.setSetpoint(targetPosition);

        motor = winch.getMotor();
        
        startTime = Util.timeStamp();
    }
    
    public void execute(boolean intrrupted){
        elapsedTime = Util.getElaspedTime(lastTimeCalled);

        lastTimeCalled = Util.timeStamp();

        power = pidCon.calculate(motor.getEncoder().getPosition(), elapsedTime);
    
        motor.set(power);
    }

    public boolean isFinished(){
        return pidCon.onTarget(tolerance);
    }

    public void end(){
        
    }
}