package Team4450.Robot23.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import Team4450.Lib.FXEncoder;
import Team4450.Lib.Util;

public class Claw {
    
    private WPI_TalonFX         clawMotor = new WPI_TalonFX(15);
    private FXEncoder           encoder = new FXEncoder(clawMotor);

    private ClawPosition        clawState;

    private final double        CLAW_MAX = 13000;
    
    /*
     * Each enum represent a predetermined position the Claw will run to
     * @param OPEN
     * @
     */
    public enum ClawPosition{
        OPEN,
        CLOSEDCONE,
        CLOSEDCUBE,
    }
    
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
        switch(desiredState){
            
            case CLOSEDCONE:
                clawMotor.set(TalonFXControlMode.Position, 2.0);
                clawState = ClawPosition.CLOSEDCONE;
                break;
            
            case CLOSEDCUBE:
                clawMotor.set(TalonFXControlMode.Position, 1.0);
                clawState = ClawPosition.CLOSEDCUBE;
                break;

            case OPEN:
                clawMotor.set(TalonFXControlMode.Position, 0.0);
                clawState = ClawPosition.OPEN;
                break;
            
        }
    }

    public ClawPosition getClawState(){
        return clawState;
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