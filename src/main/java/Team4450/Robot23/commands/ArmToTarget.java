package Team4450.Robot23.commands;

import com.revrobotics.CANSparkMax;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmToTarget extends CommandBase  {
    
    private SynchronousPID          pidCon = new SynchronousPID(0, 0, 0);

    private Arm                     arm;

    private CANSparkMax             motor;
    
    private double                  lastTimeCalled, startTime, elapsedTime;
    private double                  targetCounts, power;
    private double                  tolerance = .5;

    public ArmToTarget(Arm arm, int targetCounts){
        this.arm = arm;
        this.targetCounts = targetCounts;
        
    }

    public void initialize(){
        //below values are temporary
        pidCon.setOutputRange(0.5, 0.0);

        pidCon.setSetpoint(targetCounts);

        motor = arm.getMotor();
        
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
