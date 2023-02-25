package Team4450.Robot23.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import Team4450.Lib.Util;
import Team4450.Robot23.Constants;

public class Claw {
    
    private TalonFX         clawMotor = new TalonFX(15);

    private ClawPosition     clawState;
    
    /*
     * Each enum represent a predetermined position the Claw will run to
     * @param OPEN
     * @
     */
    public enum ClawPosition{
        OPEN,
        CLOSEDCONE,
        CLOSEDCUBE
    }
    
    public Claw(){
        Util.consoleLog("Claw created!");
    }

    public void initialize(){
        
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
}

