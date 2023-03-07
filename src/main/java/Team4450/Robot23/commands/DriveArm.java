package Team4450.Robot23.commands;

import java.util.function.DoubleSupplier;

import Team4450.Lib.Util;
import Team4450.Robot23.subsystems.Arm;
import static Team4450.Robot23.Constants.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveArm extends CommandBase 
{
    private final Arm           arm;

    private final DoubleSupplier armSupplier;

    public DriveArm(Arm arm, DoubleSupplier armSupplier)
    {
        Util.consoleLog();

        this.arm = arm;
        this.armSupplier = armSupplier;

        addRequirements(arm);
    }

    @Override
    public void execute()
    {
        double power = deadband(armSupplier.getAsDouble(), THROTTLE_DEADBAND);

        power = Util.squareInput(power);

        arm.setPower(power);
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);
    }

    private static double deadband(double value, double deadband) 
    {
        return Math.abs(value) > deadband ? value : 0.0;
    }
}
