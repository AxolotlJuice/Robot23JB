package Team4450.Robot23.commands.autonomous;

import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.math.geometry.Pose3d;

public class Auto23BlueCone2 {

    private DriveBase           driveBase;
    private Arm                 arm;
    private Winch               winch;
    private Pose3d              startingPose;

    public Auto23BlueCone2(DriveBase driveBase, Arm arm, Winch winch, Pose3d startingPose){
        this.driveBase = driveBase;
        this.arm = arm;
        this.winch = winch;
        this.startingPose = startingPose;
    }

    public void initailize(){

    }

    public void execute(){

    }

    public void isFinished(){

    }

    public void end(){

    }
    
}
