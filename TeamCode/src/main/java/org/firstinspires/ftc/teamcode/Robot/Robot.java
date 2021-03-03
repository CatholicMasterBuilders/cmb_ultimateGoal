package org.firstinspires.ftc.teamcode.Robot;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Navigation.ArcPath;
import org.firstinspires.ftc.teamcode.Navigation.EMU;
import org.firstinspires.ftc.teamcode.Navigation.EncoderTracker;
import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
import org.firstinspires.ftc.teamcode.Navigation.Maneuver;
import org.firstinspires.ftc.teamcode.Navigation.Path;

import java.lang.reflect.Method;

// TODO Make a test class to check all the wiring of the motors and servos.
public class Robot {
    // Configuration
    public final int whichMineralDetector = 0; //0 - order_detector, 1 - TensorFlow
    public final int whichCamera = 0; //0 - phone, 1 - webcam
    public final int cog_offset_from_front = (int)(10.5 * 25.4);
    public final int cog_offset_from_back = (int)(18 * 25.4) - cog_offset_from_front;
    public boolean useEMUWheels = false;
    public boolean useEMU = false;

    // Debug configuration object
    // create your own variable which can be used to turn debugging statements on/ReturnToThree
    public class DebugConfig {
        public final boolean Manuever_Trace                     = true;
        public final boolean EMU_update_trace                   = false;
        public final boolean EMU_trace                          = true;
    }

    // data handles from the running opMode
    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    // member variables
    public Drivetrain drivetrain;
    public IMU IMU;
    public EMU emu;
    public IntakeSystem intake;
    public PlacingSystem placingSystem;
    public SliderArm sliderArm;
    public Coordinate robot_coord;
    public int heading_adjustment = 0;
    public Path path;
    public ArcPath arcPath;
    public Maneuver maneuver;
    public FoundationLocks foundationlocks;
    public DebugConfig debugger;
    public Launcher launcher;

    // TAG to be used for log filtering
    public final static String DBG_TAG = "CMB_RR";

    // Constructors
    public Robot(LinearOpMode currentOpMode) {

        opMode = currentOpMode;
        hardwareMap = currentOpMode.hardwareMap;
        telemetry = currentOpMode.telemetry;
//        position = new Position();
//        velocity = new Velocity();

        // create all the member variables
        drivetrain = new Drivetrain(this );
        IMU = new IMU(this );
        emu = new EMU(this );
        intake = new IntakeSystem( this );
        path = new Path(this);
        arcPath = new ArcPath(this);
        maneuver = new Maneuver(this);
        placingSystem = new PlacingSystem(this);
//        sliderArm = new SliderArm(this);
//        foundationlocks = new FoundationLocks(this);

        debugger = new DebugConfig();
        launcher = new Launcher(this);

        robot_coord = new Coordinate(0,0, Coordinate.Direction.FORWARD);
    }

    // Initializers
    private ElapsedTime updateTimer;

    public boolean init() {

        drivetrain.drive_init();
        IMU.IMU_calibration();
        if ( useEMU ) {
            emu.init();
        }

        intake.init();
        placingSystem.init();
//        sliderArm.init();
//        foundationlocks.init();
        updateTimer = new ElapsedTime();
        updateTimer.reset();
        return true;
    }

//    public void runOnButton (boolean button, Method onAction, Object offAction){
//        boolean buttonPressed = false;
//        boolean isOff = true;
//        if (button){
//            buttonPressed = true;
//        }
//
//        if (!button && buttonPressed){
//            if(isOff) {
//                onAction;
//                isOff = false;
//            }
//
//            else if (!isOff){
//                robot.launcher.ReturnToThree();
//                isOff = true;
//            }
//            aPressed = false;
//        }
//    }
    // Methods
    //TODO Create an algorithm to track curved/not straight movement.
    public boolean updateRobotPosition(Coordinate robot_position, double last_heading, boolean throttleUpdates){
        if ( useEMU ) {
            // nothing to do
            return true;
        }
        if ( throttleUpdates ) {
            // Check to see if enough time has passed to warrant performing this update
            if ( updateTimer.milliseconds() < 100 ) {
                return true; // Nothing to do because we called this recently
            }
        }
        updateTimer.reset();
        drivetrain.encoderTracker.update();

        //Start heading via enc calculation
        double starth = last_heading;
        double deltah = 0;
        if (useEMUWheels) {
            // Update the current heading using the values from the encoders
//            last_heading += drivetrain.getHeadingDelta();
            deltah = drivetrain.getHeadingDelta();
            last_heading += deltah;
        }
        else {
            // Update the current heading to be what it was at the end of the movement instead
            // of what it was at the beginning
            last_heading = IMU.get_x_axis_heading();
        }
        //End heading via enc calculation

        double distance_traveled = 0;
        if ( drivetrain.robot_position_integrator == false ) { // This is only done once per move
            distance_traveled = drivetrain.getDistanceTravelled();
        }
        else { // This is called many times per move
            distance_traveled = drivetrain.getDistanceTravelled();
        }

        double new_robot_position_x = (Math.cos(Math.toRadians(last_heading)) * distance_traveled + robot_position.exact_x);
        double new_robot_position_y = (Math.sin(Math.toRadians(last_heading)) * distance_traveled + robot_position.exact_y);

        set_exact_robot_coord(new_robot_position_x, new_robot_position_y);

        RobotLog.dd("CMBRR", "updateRobotPosition: d = %f twdelta = (%d,%d) at heading %f + %f = %f pos (x,y) = (%d,%d)",
                distance_traveled,
                drivetrain.encoderTracker.getLeftEncoderDelta(EncoderTracker.DeltaType.INCREMENTAL),
                drivetrain.encoderTracker.getRightEncoderDelta(EncoderTracker.DeltaType.INCREMENTAL),
                starth, deltah, last_heading, robot_coord.x, robot_coord.y);

        return true;
    }

    public void set_exact_robot_coord (double x, double y) {
        robot_coord.x = (int)x;
        robot_coord.y = (int)y;
        robot_coord.exact_x = x;
        robot_coord.exact_y = y;
    }

    public void set_robot_coord (int x, int y) {
        robot_coord.x = x;
        robot_coord.y = y;
        robot_coord.exact_x = x;
        robot_coord.exact_y = y;
    }
    public Coordinate getCurrentCoordinate() {
        return robot_coord;
    }
}
