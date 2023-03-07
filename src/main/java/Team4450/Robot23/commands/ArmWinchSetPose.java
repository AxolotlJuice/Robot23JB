package Team4450.Robot23.commands;

import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ArmWinchSetPose extends CommandBase{

    private double                  radians, radius;
    private int                  targetExtend, targetRotate;
    
    private Pose2d                  targetPose;
    private Arm                     arm;
    private Winch                   winch;

    private SequentialCommandGroup	commands = null;
	private Command					command = null;

    public ArmWinchSetPose(Arm arm, Winch winch, Pose2d targetPose){
        this.arm = arm;
        this.winch = winch;
        this.targetPose = targetPose;
    }

    public void initalize(){
        
        //claculates the desired radius and rotation(radians).
        radius = targetPose.getX()/Math.acos(targetPose.getX());
        radians = Math.asin(targetPose.getY()/radius);

        //Finds the encoder counts equivalent to the radius and radians
        targetRotate = (int) (winch.getMotor().getEncoder().getCountsPerRevolution() * (radians/(2 * Math.PI)));

        targetExtend = (int) (arm.getMotor().getEncoder().getPositionConversionFactor() * radius);
        
        commands = new SequentialCommandGroup();

        commands.addCommands(new ArmToTarget(arm, targetRotate));
        
        commands.addCommands(new WinchToTarget(winch, targetExtend));

        commands.schedule();

        arm.setArmCounts(targetExtend, 0.5);
    }

    public void excute(){
        //nothing to excute
    }

    public boolean isFinished(){
        return true;
    }

    public void end(){
        
    }

}
