package Team4450.Robot23.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import Team4450.Lib.FXEncoder;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.CommandBase;
import Team4450.Robot23.Constants.*;

public class SetClawState extends CommandBase  {
    
    private SynchronousPID          pidCon = new SynchronousPID(0, 0, 0);
    private ClawPosition            targetPosi;

    private Claw                    claw;

    private WPI_TalonFX             motor;
    private FXEncoder               encoder;
    
    private double                  lastTimeCalled, startTime, elapsedTime;
    private double                  power;
    private double                  tolerance = .5;

    public SetClawState(Claw claw, ClawPosition targetPosi){
        this.claw = claw;
        this.targetPosi = targetPosi;
        
    }

    public void initialize(){
        //below values are temporary
        pidCon.setOutputRange(0.5, 0.0);

        switch(targetPosi){
        
            case CLOSEDCONE:
                pidCon.setSetpoint(13000);
                break;
            
            case CLOSEDCUBE:
                pidCon.setSetpoint(3000);
                break;

            case OPEN:
                while(motor.isRevLimitSwitchClosed() != 1){
                    claw.setPower(.20);
                }
                break;  
        }

        motor = claw.getTalonPair().getFirst();
        encoder = claw.getTalonPair().getSecond();
        
        startTime = Util.timeStamp();
    }
    
    public void execute(boolean intrrupted){
        elapsedTime = Util.getElaspedTime(lastTimeCalled);

        lastTimeCalled = Util.timeStamp();

        power = pidCon.calculate(encoder.getRotations(), elapsedTime);
    
        motor.set(power);
    }

    public boolean isFinished(){
        return pidCon.onTarget(tolerance);
    }

    public void end(){
        
    }
}
