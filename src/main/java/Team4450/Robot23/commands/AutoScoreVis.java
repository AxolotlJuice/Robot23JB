package Team4450.Robot23.commands;


import Team4450.Lib.*;

import org.opencv.core.Rect2d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.JClaw;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.LimeLight;
import Team4450.Robot23.subsystems.Winch;
import Team4450.Robot23.Constants.*;
import org.opencv.core.Rect;

import edu.wpi.first.wpilibj.Timer;


public class AutoScoreVis extends CommandBase {

    //kEndGoalY is temporary
    private static double       kP = .02, kI = .02, kD = 0, kEndGoalY = 0.0, kP2 = .02, kI2 = .02, kD2 = 0;
    private double              kToleranceDeg = .5;
   
    private double              instThrottle, instStrafe, greatestDistValue, startTime, tempTime, elapsedTime;
    private double              latestAprilTimestamp, targetToRobotDist, throttleTime, strafeTime;

    private boolean             targetLocked, hadTargets;

    private Arm                 arm;
    private Winch               winch;
    private JClaw                claw;
    private DriveBase           sDriveBase;
    private PhotonCamera        phCamera;
    private PhotonPoseEstimator phPoseEstimator;
    
    private Pose3d              latestTargetPose, targetPolePose;
    private Pose2d              latestAprilPose2d, latestRobotPose,targetPose2d;
    
    private AprilTagFieldLayout tagLayout;
    private Translation3d       limeLightToCenter;

    private SynchronousPID      pid = new SynchronousPID(kP, kI, kD);
    private SynchronousPID      pid2 = new SynchronousPID(kP2, kI2, kD2);

    private LimeLight           limeLight = new LimeLight();

    private Rect                targetTape;    
    private Preset              armTargetPose;

    private SequentialCommandGroup	commands = null;
	private Command					command = null;


    public AutoScoreVis(PhotonCamera phCamera,
                            DriveBase sDriveBase,
                            AprilTagFieldLayout tagLayout,
                            PhotonPoseEstimator phPoseEstimator,
                            Preset armTargetPose,
                            Translation3d limeLightToCenter)
    {
       
        this.phCamera = phCamera;
        this.sDriveBase = sDriveBase;
        this.phPoseEstimator = phPoseEstimator;
        this.tagLayout = tagLayout;
        this.limeLightToCenter = limeLightToCenter;

    }


    public void initialize(){
        greatestDistValue = 0.0;

        startTime = Util.timeStamp();
        tempTime = Util.timeStamp();
    }

    public void execute(){
        
        var result = phCamera.getLatestResult();

        if(claw.getClawState() == ClawPosition.CLOSEDCONE && result.hasTargets()){

            hadTargets = true;

            elapsedTime = Util.getElaspedTime(tempTime);

            //obtains the Robots pose according to the PhotonVision
            latestAprilPose2d = phPoseEstimator.update().get().estimatedPose.toPose2d();
            latestAprilTimestamp = phPoseEstimator.update().get().timestampSeconds;
            
            //Finds the target's pose, translation2d is temporary
            targetPose2d = tagLayout.getTags().get(result.getBestTarget().getFiducialId()).pose.toPose2d().transformBy(new Transform2d(new Translation2d(0.0, 3.0), new Rotation2d(0.0)));

            //merges PhotonVision pose and Odometry pose to calculate to the lateset robot pose
            sDriveBase.getOdometry().addVisionMeasurement(latestAprilPose2d, latestAprilTimestamp);
            latestRobotPose = sDriveBase.getOdometry().getEstimatedPosition();

            //claculate distance between aprilTag and robot (turn pose2d ---> pose3d)
            targetToRobotDist = Math.sqrt(Math.pow(Math.abs(targetPose2d.getX() - latestRobotPose.getX()), 2.0)
                + Math.pow(Math.abs(targetPose2d.getY() - latestRobotPose.getY()), 2.0));
            
            //get the direction the robot needs to go in
            //instThrottle = 0.5*(targetToRobotDist * Math.sin(result.getBestTarget().getYaw()));
            //instStrafe = 0.5*(targetToRobotDist * Math.cos(result.getBestTarget().getYaw()));
        
            //drive in the drection, rotation is temp
            sDriveBase.drive(pid.calculate(instThrottle, elapsedTime), pid2.calculate(instStrafe, elapsedTime),
                            0.0);
        
        }

        else if(claw.getClawState() == ClawPosition.CLOSEDCONE && limeLight.targetVisible()) {
            
            //now had targets
            hadTargets = true;

            //gets the retangle of the tape target
            targetTape = limeLight.getTargetRectangle();

            //creates a time for both pid2s
            elapsedTime = Util.getElaspedTime(tempTime);

            //Enters modified power values based on offset from tape target
            sDriveBase.drive(pid.calculate(-limeLight.offsetX(), elapsedTime), 
                             pid2.calculate(-limeLight.offsetY() - kEndGoalY, elapsedTime), 0.0);
            
        }

        else if(hadTargets){
                
            latestRobotPose = sDriveBase.getOdometry().getEstimatedPosition();

            //claculate distance between aprilTag and robot (turn pose2d ---> pose3d)
            targetToRobotDist = Math.sqrt(Math.pow(Math.abs(targetPose2d.getX() - latestRobotPose.getX()), 2.0)
                + Math.pow(Math.abs(targetPose2d.getY() - latestRobotPose.getY()), 2.0));

            //get the direction the robot needs to go in
            throttleTime = (targetToRobotDist * Math.sin(result.getBestTarget().getYaw())/0.5);
            strafeTime = (targetToRobotDist * Math.cos(result.getBestTarget().getYaw())/0.5);
            
            //drive in the drection
            sDriveBase.drive(0.5, 0.0, 0.0);
            Timer.delay(throttleTime);

            sDriveBase.drive(0.0, 0.5, 0.0);
            Timer.delay(strafeTime);

        }

        else {
            //add error message
            Util.consoleLog("No visable target & no previous targets.");
            Util.consoleLog("PhotonHasTargets:%b, LimeHasTargets:%b, HadTargets:%b, ClawPosition:%s", 
                            result.hasTargets(), limeLight.targetVisible(), hadTargets, claw.getClawState().name());
            end();
            
        }
    }

    public boolean isFinished(){

        return latestRobotPose == targetPose2d;

    }

    public void end(){
        //score with arm
        /* 
        commands = new SequentialCommandGroup();
        commands.addCommands(new ArmWinchPresets(arm, winch, armTargetPose));
        commands.schedule();
        claw.setClawState(ClawPosition.OPEN);
        */

        //resets hadTargets
        hadTargets = false;
    }
}
