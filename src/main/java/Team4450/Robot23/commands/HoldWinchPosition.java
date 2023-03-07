package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves the arm to a target position.
 */
public class HoldWinchPosition extends CommandBase 
{
    private final Winch     winch;
    private SynchronousPID  controller = new SynchronousPID(0.1, 0, 0);
    private final double    tolerance = .25, maxPower = .10;
    private double          lastTimeCalled;

    /**
     * Move winch to target position.
     * @param winch Winch subsystem.
     */
    public HoldWinchPosition(Winch winch)
    {
        Util.consoleLog();

        this.winch = winch;

        addRequirements(winch);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        controller.reset();

        controller.setSetpoint(winch.getPosition());

        controller.setOutputRange(-maxPower, maxPower);

        SmartDashboard.putBoolean("HoldWinch", true);

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
        // Note: This commands runs until canceled.
        //return controller.onTarget(tolerance); // || winch.getUpperSwitch();
        return false;
    }

    @Override
    public void end(boolean interrupted) 
    {
        winch.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("HoldWinch", false);
    }
}
