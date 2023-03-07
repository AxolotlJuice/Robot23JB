package Team4450.Robot23.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DigitalInput;

import static Team4450.Robot23.Constants.*;

public class Winch {

    private double                  elapsedTime, power;

    private final double            tolerance = .5, maxPower = .30;

    private SynchronousPID          pid = new SynchronousPID(.01, 0, 0);

    private CANSparkMax             winchMotor = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder         encoder = winchMotor.getEncoder();
    private DigitalInput            lowerLimitSwitch = new DigitalInput(WINCH_SWITCH_LOWER);
    private DigitalInput            upperLimitSwitch = new DigitalInput(WINCH_SWITCH_UPPER);

    private final double            WINCH_MAX = 116; //revolutions

    
    public Winch(){
        Util.consoleLog("Winch Created!");
    }

    public void initialize(){
        

        Util.consoleLog();

    }

    public void periodic(){
        
    }

    public void setWinchCounts(int counts, double startSpeed){
        
        pid.setSetpoint(counts);

        pid.setOutputRange(-maxPower, maxPower);

        while(pid.onTarget(tolerance)){
        
            elapsedTime = Util.getElaspedTime();

            power = pid.calculate(winchMotor.getEncoder().getPosition(), elapsedTime);

            power = Util.clampValue(power, .70);

            winchMotor.set(power);
        }
    }

    public void AutonSetWinchCounts(int counts, double startSpeed){
        
        pid.setSetpoint(counts);

        pid.setOutputRange(counts, startSpeed);

        while(pid.onTarget(tolerance)){
        
            elapsedTime = Util.getElaspedTime();

            power = pid.calculate(winchMotor.getEncoder().getPosition(), elapsedTime);

            power = Util.clampValue(power, .70);

            winchMotor.set(power);
        }

        holdPosition();
    }


    public void setWinchSpeed(double speed){
        winchMotor.set(speed);
    }

    public CANSparkMax getMotor(){
        return winchMotor;
    }

    public void holdPosition()
    {
        winchMotor.set(.10);
    }

}
