package Team4450.Robot23.subsystems;

import edu.wpi.first.math.geometry.Pose3d;

public class Pole {
    
    private int ID;
    private Pose3d pose;
    
    public Pole(int ID, Pose3d pose){
        this.ID = ID;
        this.pose = pose;
    }

    public int getID(){
        return ID;
    }

    public Pose3d getPose(){
        return pose;
    }

    
}
