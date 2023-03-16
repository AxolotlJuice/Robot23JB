package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves the arm to a target position going down.
 */
public class LowerArm extends CommandBase 
{
    private final Winch     winch;
    private double          targetPostion = 100;    // Revolutions of motor.
    private SynchronousPID  controller = new SynchronousPID(.01, 0, 0);
    private final double    tolerance = 5, maxPower = .20;
    private double          lastTimeCalled;

    /**
     * Move winch to target position going downward.
     * @param winch Winch subsystem.
     * @param targetPosition Target position in winch motor revolutions (-).
     */
    public LowerArm(Winch winch, double targetPosition)
    {
        Util.consoleLog();

        this.winch = winch;

        this.targetPostion = targetPosition;

        addRequirements(winch);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        controller.reset();

        controller.setSetpoint(targetPostion);

        controller.setOutputRange(-maxPower, maxPower);

        SmartDashboard.putBoolean("LowerArm", true);

        lastTimeCalled = Util.timeStamp();
    }

    @Override
    public void execute()
    {
        double time = Util.getElaspedTime(lastTimeCalled);

        lastTimeCalled = Util.timeStamp();

        double power = controller.calculate(winch.getPosition(), time);

        winch.setPower(power);
    }

    @Override
    public boolean isFinished()
    {
        return controller.onTarget(tolerance) || winch.getLowerSwitch();
    }

    @Override
    public void end(boolean interrupted) 
    {
        winch.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("LowerArm", false);
    }
}
