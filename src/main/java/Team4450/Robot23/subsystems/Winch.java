package Team4450.Robot23.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;

public class Winch {

    private double                  currentSpeed, elapsedTime;

    private SynchronousPID          pid = new SynchronousPID(0, 0, 0);

    private CANSparkMax             winchMotor = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
    
    public Winch(){
        Util.consoleLog("Winch Created!");
    }

    public void initialize(){
        
        winchMotor.getEncoder().getPosition();

        Util.consoleLog();
    }

    public void periodic(){
        
    }

    public void setWinchCounts(int counts, double startSpeed){
        
        pid.setSetpoint(counts);

        while(winchMotor.getEncoder().getVelocity() != 0.0){
        
        elapsedTime = Util.getElaspedTime();

        currentSpeed = pid.calculate(winchMotor.getEncoder().getPosition(), elapsedTime);

        winchMotor.set(currentSpeed);
        }
    }

    public void setWinchSpeed(double speed){
        winchMotor.set(speed);
    }

    public CANSparkMax getMotor(){
        return winchMotor;
    }

}
