package Team4450.Robot23.commands;

import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmWinchPresets extends CommandBase{

    private Arm             arm;
    private Winch           winch;
    private Pose2d          targetPose;
    private Preset          preset;

    private SequentialCommandGroup	commands = null;
	private Command					command = null;

    private ArmWinchSetPose         poseSetter;

    public enum Preset{
        GRABBING,
        POLEHIGH,
        POLELOW,
        TAGHIGH,
        TAGLOW
    }
    
    public ArmWinchPresets(Arm arm, Winch winch, Pose2d targetPose, Preset preset){
        this.arm = arm;
        this.winch = winch;
        this.preset = preset;
    }

    public void initailize(){
        
        switch(preset){
            case GRABBING:
                winch.getMotor().getEncoder().setPosition(0.0);
                arm.getMotor().getEncoder().setPosition(0.0);

            case POLEHIGH:
                commands = new SequentialCommandGroup();
                command = new ArmWinchSetPose(arm, winch, new Pose2d(39.7256, 109.9873, null));
                commands.addCommands(command);
                commands.schedule();

            case POLELOW:
                commands = new SequentialCommandGroup();
                command = new ArmWinchSetPose(arm, winch, new Pose2d(22.7125, 81.0717, null));
                commands.addCommands(command);
                commands.schedule();

            case TAGHIGH:
                commands = new SequentialCommandGroup();
                command = new ArmWinchSetPose(arm, winch, new Pose2d(22.7125, 81.0717, null));
                commands.addCommands(command);
                commands.schedule();

            case TAGLOW:
                commands = new SequentialCommandGroup();
                command = new ArmWinchSetPose(arm, winch, new Pose2d(22.7125, 81.0717, null));
                commands.addCommands(command);
                commands.schedule(); 
            
        }
    }

    public void execute(){

    }

    public boolean isFinished(){
        return true;
    }

    public void end(){

    }
}