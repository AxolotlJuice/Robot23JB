package Team4450.Robot23.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class JWinch extends SubsystemBase{

    private double                  elapsedTime, power;

    private final double            tolerance = .5, maxPower = .30;

    private SynchronousPID          pid = new SynchronousPID(.01, 0, 0);

    private CANSparkMax             winchMotor = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
    private RelativeEncoder         encoder = winchMotor.getEncoder();
    private DigitalInput            lowerLimitSwitch = new DigitalInput(WINCH_SWITCH_LOWER);
    private DigitalInput            upperLimitSwitch = new DigitalInput(WINCH_SWITCH_UPPER);

    private final double            WINCH_MAX = 116; //revolutions

    
    public JWinch(){
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

    public void stop()
    {
        winchMotor.stopMotor();
    }

    public void setPower(double power){
        winchMotor.set(power);
    }

    public CANSparkMax getMotor(){
        return winchMotor;
    }

    public double getPosition(){
        return winchMotor.getEncoder().getPosition();
    }

    public void holdPosition()
    {
        winchMotor.set(.10);
    }
//-----------------------------------------------------------------------------------------------------

    /**
     * Returns state of lower position limit switch.
     * @return True is at low position.
     */
    public boolean getLowerSwitch()
    {
        return lowerLimitSwitch.get();
    }

    /**
     * Returns state of pperr position limit switch.
     * @return True is at high position.
     */
    public boolean getUpperSwitch()
    {
        return upperLimitSwitch.get();
    }
}
