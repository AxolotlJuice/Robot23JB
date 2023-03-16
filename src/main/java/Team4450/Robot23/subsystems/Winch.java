package Team4450.Robot23.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static Team4450.Robot23.Constants.*;

public class Winch  extends SubsystemBase
{
    private CANSparkMax     motor = new CANSparkMax(WINCH_MOTOR, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private DigitalInput    lowerLimitSwitch = new DigitalInput(WINCH_SWITCH_LOWER);
    private DigitalInput    upperLimitSwitch = new DigitalInput(WINCH_SWITCH_UPPER);

    private final double    WINCH_MAX = 116;    // Revolutions.

    public Winch()
    {
        Util.consoleLog();

        // Winch will start at max up position so that is encoder zero. Encoder max will
        // be winch at lowest position.

        encoder.setPosition(0);
    }

    /**
     * Set winch power.
     * @param power + is up, - is down.
     */
    public void setPower(double power)
    {
        // If power negative, which means go down, check limit switch stop if true.
        // If power positive, which means go up, check limit for max height, stop if true.

        if ((power < 0 && lowerLimitSwitch.get()) || (power > 0 && upperLimitSwitch.get())) power = 0;
        
        //if (power > 0 && upperLimitSwitch.get()) power = 0;

        //if ((power < 0 && encoder.getPosition() >= WINCH_MAX) || (power > 0 && encoder.getPosition() <= 0)) power = 0;

        if (upperLimitSwitch.get() || lowerLimitSwitch.get()) encoder.setPosition(0);

        power = Util.clampValue(power, .70);

        motor.set(power);
    }

    public void stop()
    {
        motor.stopMotor();
    }

    /**
     * Return Winch encoder position. Note winch encoder counts up (+) when power
     * is + (going up) and down (-) when power is - (going down).
     * @return Position in revolutions.
     */
    public double getPosition()
    {
        return encoder.getPosition();
    }

    /**
     * Reset Winch encoder to zero.
     */
    public void resetPosition()
    {
        encoder.setPosition(0);
    }

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

    public void holdPosition()
    {
        setPower(.10);
    }

    public void updateDS()
    {

    }
}
