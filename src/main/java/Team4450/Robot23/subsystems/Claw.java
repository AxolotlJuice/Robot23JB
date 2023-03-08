package Team4450.Robot23.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import Team4450.Lib.FXEncoder;
import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.Constants.*;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    
    private WPI_TalonFX                     clawMotor = new WPI_TalonFX(15);
    private FXEncoder                       encoder = new FXEncoder(clawMotor);

    private SynchronousPID                  pid = new SynchronousPID(0, 0, 0);

    private ClawPosition                    clawState;

    private double                          power, time, lastTimeCalled;

    private Pair<WPI_TalonFX, FXEncoder>    talonPair = new Pair(clawMotor, encoder);

    private final double                    CLAW_MAX = 13000, tolerance = 1000;

    
    
    /*
     * Each enum represent a predetermined position the Claw will run to
     * @param OPEN
     * @
     */
    
    public Claw(){
        Util.consoleLog("Claw created!");
    }

    public void initialize(){

        clawMotor.setInverted(true);

        encoder.setInverted(true);

        clawMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                                             LimitSwitchNormal.Disabled,
                                             30);

        clawMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                                             LimitSwitchNormal.Disabled,
                                             30);
        
        setClawState(ClawPosition.OPEN);
        
        Util.consoleLog();
    }

    public void periodic(){
        //this method is called once per schedular run
    }

    public void setClawState(ClawPosition desiredState){
        //temp values
        
        pid.setOutputRange(-.20, .20);
        
        double time = Util.getElaspedTime(lastTimeCalled);
        lastTimeCalled = Util.timeStamp();

        switch(desiredState){
            
            case CLOSEDCONE:
                pid.setSetpoint(13000);

                while(pid.onTarget(tolerance)){
                    time = Util.getElaspedTime(lastTimeCalled);
                    power = pid.calculate(encoder.getAbsolutePosition(), time);
                    clawMotor.set(power);
                    clawState = ClawPosition.CLOSEDCONE;
                }
                break;
            
            case CLOSEDCUBE:
                while(pid.onTarget(tolerance)){
                    pid.setSetpoint(3000);

                    time = Util.getElaspedTime(lastTimeCalled);
                    power = pid.calculate(encoder.getAbsolutePosition(), time);
                    clawMotor.set(power);
                    clawState = ClawPosition.CLOSEDCUBE;
                }
                break;

            case OPEN:
                while(clawMotor.isRevLimitSwitchClosed() != 1){
                    setPower(.20);
                }
                break;
            
        }
    }

    public ClawPosition getClawState(){
        return clawState;
    }

    public Pair<WPI_TalonFX, FXEncoder> getTalonPair(){
        return talonPair; 
    }


//----------------------------------------------------------------------------------------------------------------------



    public void setPower(double power)
    {
        // If power positive, which means open, check limit switch stop if true.
        // If power negative, which means close, check encoder for max open, stop if there.

        if ((power > 0 && getOpenSwitch()) || (power < 0 && encoder.get() >= CLAW_MAX)) power = 0;

        if (getOpenSwitch()) encoder.reset();

        power = Util.clampValue(power, .20);
        
        clawMotor.set(power);
   }

   public boolean getOpenSwitch()
    {
        if (clawMotor.isRevLimitSwitchClosed() == 1)
            return false;
        else
            return true;
    }
}