package Team4450.Robot23.commands;


import Team4450.Lib.*;
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
import edu.wpi.first.wpilibj2.command.CommandBase;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.LimeLight;
import Team4450.Robot23.Constants;


public class AutoAimVision extends CommandBase {


    private static double       kP = .02, kI = .02, kD = 0;
    private double              kToleranceDeg = .5;
    
    private boolean             targetLocked, hadTargets;
    private boolean             cubeInClaw;
   
    private double              instThrottle;
    private double              instStrafe;
    private double              greatestDistValue;
    private double              startTime;
    private double              tempTime;
    private double              elapsedTime;


    private DriveBase           sDriveBase;
    private PhotonCamera        phCamera;
    private PhotonPoseEstimator phPoseEstimator;
   
    private double              latestAprilTimestamp;
    private double              targetToRobotDist;
    
    private Pose3d              latestTargetPose;
    private Pose3d              targetPolePose;
    private Pose2d              latestAprilPose2d;
    private Pose2d              latestRobotPose;
    private Pose2d              targetPose2d;
    
    private AprilTagFieldLayout tagLayout;
    private Translation2d       limeLightToCenter;

    private SynchronousPID          pid = new SynchronousPID(kP, kI, kD);

    private LimeLight               limeLight = new LimeLight();


    public AutoAimVision(PhotonCamera phCamera,
                            DriveBase sDriveBase,
                            AprilTagFieldLayout tagLayout,
                            PhotonPoseEstimator phPoseEstimator,
                            Translation2d limeLightToCenter)
    {
       
        this.phCamera = phCamera;
        this.sDriveBase = sDriveBase;
        this.phPoseEstimator = phPoseEstimator;
        this.tagLayout = tagLayout;
        this.limeLightToCenter = limeLightToCenter;
        this.cubeInClaw = false;

    }

    public AutoAimVision(PhotonCamera phCamera,
                            DriveBase sDriveBase,
                            AprilTagFieldLayout tagLayout,
                            PhotonPoseEstimator phPoseEstimator,
                            Translation2d limeLightToCenter, 
                            boolean cubeInClaw)
    {
       
        this.phCamera = phCamera;
        this.sDriveBase = sDriveBase;
        this.phPoseEstimator = phPoseEstimator;
        this.tagLayout = tagLayout;
        this.limeLightToCenter = limeLightToCenter;
        this.cubeInClaw = cubeInClaw;

    }


    public void initialize(){
        greatestDistValue = 0.0;

        startTime = Util.timeStamp();
        tempTime = Util.timeStamp();
    }

    public void execute(){
        
        if(cubeInClaw){
            var result = phCamera.getLatestResult();
            
            if(result.hasTargets()){

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
                instThrottle = 0.5*(targetToRobotDist * Math.sin(result.getBestTarget().getYaw()));
                instStrafe = 0.5*(targetToRobotDist * Math.cos(result.getBestTarget().getYaw()));
            
                //drive in the drection, rotation is temp
                sDriveBase.drive(pid.calculate(instThrottle, elapsedTime), pid.calculate(instStrafe, elapsedTime),
                                0.0);
            }
            else if(hadTargets){
                
                latestRobotPose = sDriveBase.getOdometry().getEstimatedPosition();

                //claculate distance between aprilTag and robot (turn pose2d ---> pose3d)
                targetToRobotDist = Math.sqrt(Math.pow(Math.abs(targetPose2d.getX() - latestRobotPose.getX()), 2.0)
                    + Math.pow(Math.abs(targetPose2d.getY() - latestRobotPose.getY()), 2.0));
                
                if(targetToRobotDist > greatestDistValue)
                    greatestDistValue = targetToRobotDist;

                //get the direction the robot needs to go in
                instThrottle = 0.5*(targetToRobotDist * Math.sin(result.getBestTarget().getYaw()))/greatestDistValue;
                instStrafe = 0.5*(targetToRobotDist * Math.cos(result.getBestTarget().getYaw()))/greatestDistValue;
            
                //drive in the drection
                
                sDriveBase.drive(instThrottle, instStrafe, 0.0);
            }
            else {
                end();
            }
        }
        else {

            hadTargets = true;

            var result = phCamera.getLatestResult();

            if(limeLight.targetVisible()){
                //targetPolePose = Constants.poleLayout.get(2 * result.getBestTarget().getFiducialId() - (if(result.getBestTarget().getYaw() > 0) ? 1 : 0)).getPose();

                //line up the pixels, x = strafe & y = throttle
            }
            else {
                end();
            }
        }
    }

    public boolean isFinished(){
        return latestRobotPose == targetPose2d;
    }


    public void end(){
        //score with arm
        
        //resets hadTargets
        hadTargets = false;
    }
}
