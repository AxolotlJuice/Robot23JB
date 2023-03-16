package Team4450.Robot23.commands;

import Team4450.Lib.SynchronousPID;
import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Claw;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Moves the Claw to a target position.
 */
public class CloseClaw extends CommandBase 
{
    private final Claw      claw;
    private double          targetPostion = 3000;    // Encoder tick count.
    private SynchronousPID  controller = new SynchronousPID(.0001, .00001, 0);
    private final double    tolerance = 1000, maxPower = .20;
    private double          lastTimeCalled;

    /**
     * Move claw to target position (closing it).
     * @param claw Claw subsystem.
     * @param targetPosition Target position in encoder tick counts.
     */
    public CloseClaw(Claw claw, double targetPosition)
    {
        Util.consoleLog();

        this.claw = claw;

        this.targetPostion = targetPosition;

        addRequirements(claw);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();

        controller.reset();

        controller.setSetpoint(targetPostion);

        controller.setOutputRange(-maxPower, maxPower);

        SmartDashboard.putBoolean("CloseClaw", true);

        lastTimeCalled = Util.timeStamp();
    }

    @Override
    public void execute()
    {
        double time = Util.getElaspedTime(lastTimeCalled);

        lastTimeCalled = Util.timeStamp();

        // Note that the encoder will count + from fully open position
        // where it is reset. This results in the PID controller generating
        // a + power value. However, - power is required to close the claw.
        // Hence the inversion of the power below.

        double power = controller.calculate(claw.getPosition(), time);

        claw.setPower(-power);
    }

    @Override
    public boolean isFinished()
    {
        return controller.onTarget(tolerance);
    }

    @Override
    public void end(boolean interrupted) 
    {
        claw.stop();

        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putBoolean("CloseClaw", false);
    }
}
