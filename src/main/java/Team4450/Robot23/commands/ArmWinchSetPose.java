package Team4450.Robot23.commands;

import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Winch;
import edu.wpi.first.math.geometry.Pose2d;

public class ArmWinchSetPose {

    private double          radians, radius, targetExtend, targetRotate;
    
    private Pose2d          targetPose;
    private Arm             arm;
    private Winch           winch;

    public ArmWinchSetPose(Arm arm, Winch winch, Pose2d targetPose){
        this.arm = arm;
        this.winch = winch;
        this.targetPose = targetPose;
    }

    public void initalize(){

        //claculates the desired radius and rotation(radians)
        radius = targetPose.getX()/Math.acos(targetPose.getX());
        radians = Math.asin(targetPose.getY()/radius);

        //Finds the encoder counts equivalent to the radius and radians
        targetRotate = winch.getMotor().getEncoder().getCountsPerRevolution() * (radians/(2 * Math.PI));

        targetExtend = arm.getMotor().getEncoder().getPositionConversionFactor() * radius;
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
