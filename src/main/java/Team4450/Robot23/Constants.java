
package Team4450.Robot23;

import java.util.Arrays;
import java.util.Properties;

import Team4450.Lib.Util;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static String		PROGRAM_NAME = "ORF23-02.15.23-1";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false, tracing = false;
	    	
	public static DriverStation.Alliance	alliance;
	public static int                       location, matchNumber;
	public static String					eventName, gameMessage;
	    
	// Non-drive base motor controller port assignments

    public static final int     CLAW_MOTOR = 15;
    public static final int     WINCH_MOTOR = 13;
    public static final int     ARM_MOTOR = 14;
    public static final int     REV_PDB = 20;
	
	// GamePad port assignments.
	public static final int		DRIVER_PAD = 0, UTILITY_PAD = 1;

	// Pneumatic valve controller port assignments.
	//public static final int		COMPRESSOR = 0;

	// Digital Input port assignments. Encoder takes 2 ports.
    public static final int     WINCH_SWITCH = 0;
    public static final int     ARM_SWITCH = 1;
    public static final int     CLAW_SWITCH = 2;     
	  
	// Analog Input port assignments.
	
	// LCD display line number constants showing class where the line is set.
	public static final int		LCD_1 = 1;	    // Robot, Auto Commands.
	public static final int		LCD_2 = 2;	    // Swerve Drive command.
	public static final int		LCD_3 = 3;	    // ShuffleBoard subsystem.
	public static final int		LCD_4 = 4;	    // ShuffleBoard subsystem.
	public static final int		LCD_5 = 5;	    // Autonomous commands.
	public static final int		LCD_6 = 6;	    // ShuffleBoard subsystem.
	public static final int		LCD_7 = 7;	    // ShuffleBoard subsystem.
	public static final int		LCD_8 = 8;	    // ShuffleBoard subsystem.
	public static final int		LCD_9 = 9;	    // ShuffleBoard subsystem.
	public static final int		LCD_10 = 10;	// ShuffleBoard subsystem.

	// Default starting field position in meters for pose tracking.
	public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(2.610, 2.847, new Rotation2d(0));
    
	// Next group of constants are for Swerve drive.

    // The maximum voltage that will be delivered to the drive motors. This can be reduced to cap the 
    // robot's maximum speed. Typically, this is useful during initial testing of the robot.
   
	public static final double MAX_VOLTAGE = 6.0; //12.0;

	public static final double  THROTTLE_DEADBAND = .05;
    public static final double  ROTATION_DEADBAND = .05;

    // Slew is rate of change per second in whatever unit you are using.
    // We are doing % stick input so unit is 100% or 1. So a slew of i is
    // ramp to 1 (100% power) in 1 second. So a slew of 3 is 3 units per
    // second or 1 unit in 1/3 second. So larger slew is faster ramp to
    // 100%. 1.5 is 100% in 3/4 second. 2 is 100% in 1/2 second.
    public static final double  THROTTLE_SLEW = 1.5;        
    public static final double  ROTATION_SLEW = 2.0;  //3.0;

    /**
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Util.inchesToMeters(18.75); //  Measure and set trackwidth
   
    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Util.inchesToMeters(30.75); // Measure and set wheelbase

    // Swerve Module motor controller & encoder port assignments and steering offsets.

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; // Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; //  Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; //  Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(301.113); //  Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; //  Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; //  Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6; //  Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(238.096); //  Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; //  Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; //  Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; //  Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(55.283); //  Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10; //  Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11; //  Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; //  Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(157.764); //  Measure and set back right steer offset
    
	// Use these values in PathWeaver for speed and acceleration.
    // Robot will go faster than this, more like 3 mps but this value tones down autonomous speed.

    public static final double  MAX_WHEEL_SPEED = 2.0;     // Meters per second.
    public static final double  MAX_WHEEL_ACCEL = 1.0;     // Meters per second per second.
    
    // Estimated by eyeball observation. Needs to be estimated each new robot.

    public static final double  MAX_ROTATIONAL_VEL = 70;    // Degrees per second.
    public static final double  MAX_ROTATIONAL_ACCEL = 70;  // Degrees per second per second.

    // Drive base characterization results. These values from 2021 as placeholders until 202?
	// characterization is done.

    //public static final double  TRACK_WIDTH_C = Util.inchesToMeters(DRIVETRAIN_TRACKWIDTH_METERS); // Meters.

    // public static final double  DB_KS = 1.74;
    // public static final double  DB_KV = 1.8;
    // public static final double  DB_KA = .422;

    // public static final double  DB_POSITIONAL_KP = .0688; 
    // public static final double  DB_POSITIONAL_KD = 36.5; 
    // public static final double  DB_VELOCITY_KP = .12;  
    // public static final double  DB_VELOCITY_KD = 0.0;

    private final AprilTagFieldLayout aprlFieldLayout = new AprilTagFieldLayout(Arrays.asList(
    new AprilTag(1, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(2, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(3, new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(4, new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, Math.PI))),
    new AprilTag(5, new Pose3d(Units.inchesToMeters( 14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(6, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(7, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0))),
    new AprilTag(8, new Pose3d(Units.inchesToMeters( 40.45), Units.inchesToMeters( 42.19), Units.inchesToMeters(18.22), new Rotation3d(0.0, 0.0, 0.0)))
  ), Units.inchesToMeters(651.25), Units.inchesToMeters(315.5));

  //-------------------- No student code above this line ------------------------------------------------------

}
