package Team4450.Robot23.commands;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj2.command.CommandBase;
import Team4450.Robot23.subsystems.DriveBase;

public class SetToStartPositionCommand extends CommandBase
{
    private final DriveBase     driveBase;

    private double              startTime;

    public SetToStartPositionCommand(DriveBase driveBase) 
    {
        Util.consoleLog();

        this.driveBase = driveBase;

        addRequirements(driveBase);
    }
    
    @Override
    public void initialize() 
    {
        Util.consoleLog();
    
        startTime = Util.timeStamp();

        driveBase.setModulesToStartPosition();;
    }
    
    @Override
    public boolean isFinished() 
    {
        if (Util.getElaspedTime(startTime) > 2.0) return true;

        return false;
    }

    @Override
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
