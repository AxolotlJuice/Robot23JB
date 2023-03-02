package Team4450.Robot23.commands.autonomous;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import Team4450.Robot23.Constants;
import Team4450.Robot23.commands.AutoScoreVis;
import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Claw;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto23BlueCone2 extends CommandBase{

    private DriveBase           driveBase;
    private Arm                 arm;
    private Winch               winch;
    private Claw                claw;

    private PhotonCamera        phCamera;
    private PhotonPoseEstimator phPoseEstimator;
    
    private Pose3d              startingPose;

    private SequentialCommandGroup	commands = null;
    private Command					command = null;

    public Auto23BlueCone2(DriveBase driveBase, Arm arm, Winch winch, PhotonCamera phCamera, Pose3d startingPose, PhotonPoseEstimator phPoseEstimator){
        this.driveBase = driveBase;
        this.arm = arm;
        this.winch = winch;
        this.startingPose = startingPose;
    }

    public void initailize(){
        commands = new SequentialCommandGroup(command);

        command = new AutoScoreVis(phCamera, driveBase, 
                                    Constants.APRILTAGFIELDLAYOUT, 
                                    PhotonPoseEstimator phPoseEstimator, 
                                    claw.getClawState(), 
                                    Constants.LIMETOCENTER);
    }

    public void execute(){

    }

    public boolean isFinished(){
        return true;
    }

    public void end(){

    }
    
}
