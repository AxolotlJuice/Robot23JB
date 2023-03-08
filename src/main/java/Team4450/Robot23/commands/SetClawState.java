package Team4450.Robot23.commands;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import Team4450.Lib.FXEncoder;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Claw;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetClawState extends CommandBase  {
    
    private SynchronousPID          pidCon = new SynchronousPID(0, 0, 0);

    private Claw                    claw;

    private WPI_TalonFX             motor;
    private FXEncoder               encoder;
    
    private double                  lastTimeCalled, startTime, elapsedTime;
    private double                  targetPosition, power;
    private double                  tolerance = .5;

    public SetClawState(Claw claw, int targetPosition){
        this.claw = claw;
        this.targetPosition = targetPosition;
        
    }

    public void initialize(){
        //below values are temporary
        pidCon.setOutputRange(0.5, 0.0);

        pidCon.setSetpoint(targetPosition);

        motor = claw.getTalonPair().getFirst();
        encoder = claw.getTalonPair().getSecond();
        
        startTime = Util.timeStamp();
    }
    
    public void execute(boolean intrrupted){
        elapsedTime = Util.getElaspedTime(lastTimeCalled);

        lastTimeCalled = Util.timeStamp();

        power = pidCon.calculate(encoder.get, elapsedTime);
    
        motor.set(power);
    }

    public boolean isFinished(){
        return pidCon.onTarget(tolerance);
    }

    public void end(){
        
    }
}
