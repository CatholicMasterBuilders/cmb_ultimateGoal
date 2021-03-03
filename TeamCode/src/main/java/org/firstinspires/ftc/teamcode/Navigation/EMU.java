package org.firstinspires.ftc.teamcode.Navigation;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DataTypes.Heading;
import org.firstinspires.ftc.teamcode.NaiveAccelerationIntegrator;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.opencv.core.Mat;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.DataTypes.Heading.toCompassHeading;
import static org.firstinspires.ftc.teamcode.DataTypes.Heading.toNormalizedHeading;
import static org.firstinspires.ftc.teamcode.Navigation.EncoderTracker.DeltaType;

/**
 * An Encoder based version of an IMU
 * Note that it requires the start heading to be initialized using setHeadingRefPoint
 */
public class EMU implements Runnable {

    Robot robot;
    Orientation angles;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;
    DcMotor left_encoder;
    DcMotor right_encoder;
    EncoderTracker encoderTracker;

    private volatile boolean isThreadRunning = true;
    private int refreshRate = 20; // Hz
    private int sleepTime = 1000 / refreshRate;
    private double currentHeading;

    // Constants related to the encoder tracking wheels
    private static final double CPR = 360*4; // For am-3132 optical encoder
    private static final double DIAMETER = 50; // Units in mm for am-3955
    private static final double CIRCUMFERENCE = DIAMETER * Math.PI;
    private static final double COUNTS_PER_MM = CPR / CIRCUMFERENCE;

    private static final double TRACK_MM = 155/*6.5*25.4*/; // Obtained from EMUCalibration
    private static final double TICK_DIFFERENTIAL_PER_ONE_DEGREE_OF_HEADING_DELTA =
            2 * Math.PI * TRACK_MM * COUNTS_PER_MM / 360;
    private static final double TICK_DIFFERENTIAL_PER_ONE_RADIAN_OF_HEADING_DELTA =
            TRACK_MM * COUNTS_PER_MM;


    public EMU(Robot robot) {
        setRobot(robot);
    }

    public void init (){
        opMode = getRobot().opMode;
        hardwareMap = getRobot().hardwareMap;
        telemetry = getRobot().telemetry;

        // Get encoders from the hardware map
//        left_encoder = hardwareMap.get(DcMotor.class, "left_front");
//        right_encoder = hardwareMap.get(DcMotor.class, "right_front")
        left_encoder = robot.drivetrain.left_rear;
        right_encoder = robot.drivetrain.right_rear;
        // Assume that drivetrain has initialized the encoders

        // Use an Encoder Tracker but this assumes they are not getting reset
        encoderTracker = new EncoderTracker( left_encoder, right_encoder );
        encoderTracker.init();
    }
    public void start( double current_heading ) {
        // Start the update thread
        isThreadRunning = true;
        setHeadingRefPoint( current_heading );
        Thread emuThread = new Thread(this );
        emuThread.start();
    }
    public void stop() {
        // Kill the running update thread
        isThreadRunning = false;
    }


    private void setHeadingRefPoint( double ref_heading ) {
        // convert to radians to avoid having to do it on every udate interval
        currentHeading = Math.toRadians( ref_heading );
    }

    private double distance;

    private void update() {
        //Get Current Positions
        encoderTracker.update();

        // Determine the new heading using the difference between the encoder ticks
        currentHeading += encoderTracker.getDifference(DeltaType.INCREMENTAL) / TICK_DIFFERENTIAL_PER_ONE_RADIAN_OF_HEADING_DELTA;

        // clean up the current heading if we wrapped around the x-axis
        currentHeading = toNormalizedHeading( Heading.Type.RADIANS, currentHeading );

        // Determine how far the robot moved on that new heading
        distance = encoderTracker.getAverage( DeltaType.INCREMENTAL ) / COUNTS_PER_MM;

        // Calculate the new robot coordinates
        robot.robot_coord.x += distance * Math.cos( currentHeading );
        robot.robot_coord.y += distance * Math.sin( currentHeading );

        // Debug logs
        if ( robot.debugger.EMU_update_trace ) {
            RobotLog.dd( robot.DBG_TAG,"EMU::update currentHeading %f, pos(x,y) = (%d,%d)\n",
                    Math.toDegrees(currentHeading), robot.robot_coord.x, robot.robot_coord.y );
        }
    }

    /**
     * This API determines the difference between the current heading and the final heading.  All
     * answers will be in the normalized range [-180,180] which also means they will be the
     * smallest angle needed to get to final_heading
     * @param final_heading the desired heading to use as a reference
     * @return the smallest angle between current and final
     */
    public double getHeadingDeltaFromCurrent( double final_heading ) {
        // Check that the provided heading is valid
        if ( final_heading <= -180 || final_heading > 180 ) {
            RobotLog.dd(robot.DBG_TAG, "EMU:getHeadingDeltaFromCurrent: invalid heading %f\n", final_heading);
            final_heading = toNormalizedHeading(final_heading);
        }

        double start_heading = get_x_axis_heading();
        double turn_distance = toNormalizedHeading( final_heading - start_heading );
        return turn_distance;
    }

    /** returns the raw heading value from the EMU.  This is guaranteed to be in the range [0,360]
     * and it is relative to the field coordinate axes
     * */
    public double get_compass_heading (){
        // The emu gives values in the range of (-179,180] but we want the range to be [0,360]
        return toCompassHeading( get_x_axis_heading() );
    }

    /** returns the raw heading value from the EMU.  This is the same as the field relative heading
     * */
    public double get_raw_heading (){
        return get_x_axis_heading();
    }

    /**
     * Get the field relative heading.  x axis is 0, heading
     * increases in CCW direction and range is [-179,180]
     * @return
     */
    public double get_x_axis_heading (boolean debug){
        // Nothing to do here, the EMU always has the field reletive heading because we can
        if ( robot.debugger.EMU_trace && debug )
            RobotLog.dd(robot.DBG_TAG, "EMU:get_x_axis_heading: %f", Math.toDegrees(currentHeading));

        return Math.toDegrees(currentHeading);
    }
    public double get_x_axis_heading (){
        return get_x_axis_heading(false);
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isThreadRunning) {
            update();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    // Getters and Setters
    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    // **************************************************************************
    // Testing/Calibrating functions
    // **************************************************************************

    /**
     * Calculate the track width when you know the encoder difference for a given rotation
     * @param encoder_differential The measured encoder difference
     * @param degrees The measured degrees
     * @return track width ( in mm )
     */
    public double calculateTrackWidth( int encoder_differential, double degrees ) {
        return ((double)(360 * encoder_differential )) / ( 2 * Math.PI * COUNTS_PER_MM * degrees );
    }

    public int getLeftEncoderValue() {
        return left_encoder.getCurrentPosition();
    }
    public int getRightEncoderValue() {
        return right_encoder.getCurrentPosition();
    }

    public void test() {
        RobotLog.dd(robot.DBG_TAG, "Start Heading(deg): %f", Math.toDegrees(currentHeading));
        RobotLog.dd(robot.DBG_TAG, "Start Heading(rad): %f", currentHeading);

        // Test: Verify the direction of the encoders
        RobotLog.dd(robot.DBG_TAG, "***Test direction of encoders***");
        RobotLog.dd(robot.DBG_TAG, "Pausing 5 seconds to push the robot ahead by hand.");
                RobotLog.dd(robot.DBG_TAG, "Verify that the encoders increase in value");
        int left_start = getLeftEncoderValue();
        int right_start = getRightEncoderValue();
        opMode.sleep( 5000 );
        int left_end = getLeftEncoderValue();
        int right_end = getRightEncoderValue();
        RobotLog.dd(robot.DBG_TAG, "Encoder Counts  Left(s,e): %d, %d", left_start, left_end);
        RobotLog.dd(robot.DBG_TAG, "Encoder Counts Right(s,e): %d, %d", right_start, right_end);
        RobotLog.dd(robot.DBG_TAG, "Encoder Counts Delta(l,r): %d, %d", left_end - left_start, right_end - right_start);

        // Test: Verify the update function with no reference heading
        int sleepThreshold = 10; // This should be less than the refreshRate for update
        RobotLog.dd(robot.DBG_TAG, "***Test location update with start heading of 0***");
        RobotLog.dd(robot.DBG_TAG, "Looping 10 seconds to push the robot by hand");

        opMode.sleep( 2000 );


        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        start(0);
        while ( timer.seconds() < 10 ) {
            RobotLog.dd(robot.DBG_TAG, "robot location = (%d, %d, %.3f)",
                    robot.robot_coord.x, robot.robot_coord.y, Math.toDegrees(currentHeading));
            opMode.sleep( sleepThreshold );
        }
        stop();

        // Test: Verify the update function with starting reference heading
        RobotLog.dd(robot.DBG_TAG, "***Test location update with start heading of 90***");
        RobotLog.dd(robot.DBG_TAG, "Resetting EMU to test with 90deg reference_heading");
        RobotLog.dd(robot.DBG_TAG, "Looping 10 seconds to push the robot by hand");
        RobotLog.dd(robot.DBG_TAG, "Verify the robot location matches your input");
        opMode.sleep( 2000 );

        timer.reset();
        robot.robot_coord.updateLocation(0,0);
        start(90);
        while ( timer.seconds() < 10 ) {
            RobotLog.dd(robot.DBG_TAG, "robot location = (%d, %d, %.3f)",
                    robot.robot_coord.x, robot.robot_coord.y, Math.toDegrees(currentHeading));
            opMode.sleep( sleepThreshold );
        }
        stop();

        // Test: Verify the various heading methods
        RobotLog.dd(robot.DBG_TAG, "***Test heading APIs***");
        RobotLog.dd(robot.DBG_TAG, "Resetting EMU to test heading methods");
        RobotLog.dd(robot.DBG_TAG, "Looping 10 seconds to turn the robot by hand");
        RobotLog.dd(robot.DBG_TAG, "Verify the heading readouts should all be the same");
        opMode.sleep( 2000 );

        timer.reset();
        start(0);
        RobotLog.dd(robot.DBG_TAG, "XHeading,RawHeading,CompassHeading");
        while ( timer.seconds() < 10 ) {
            RobotLog.dd(robot.DBG_TAG, "%.3f, %.3f, %.3f)",
                    get_x_axis_heading(false),
                    get_raw_heading(),
                    get_compass_heading());
            opMode.sleep( sleepThreshold );
        }
        stop();

        // Test: Verify the various heading methods
        RobotLog.dd(robot.DBG_TAG, "***Test heading delta calculations ***");
        RobotLog.dd(robot.DBG_TAG, "Resetting EMU to test heading delta");
        RobotLog.dd(robot.DBG_TAG, "Looping 10 seconds to turn the robot by hand");
        RobotLog.dd(robot.DBG_TAG, "Every turn of the robot will calculate the delta for all 360 degrees");
        RobotLog.dd(robot.DBG_TAG, "Verify the correct wrap arounds");
        opMode.sleep( 200000 );

        timer.reset();
        start(0);
        RobotLog.dd(robot.DBG_TAG, "Ref,Rotation,Delta");
        double delta, ref;
        while ( timer.seconds() < 10 ) {
            ref = get_x_axis_heading( false );
            for ( int i = 0; i <=360; i++ ) {
                delta = getHeadingDeltaFromCurrent(i);
                RobotLog.dd(robot.DBG_TAG, "%.3f, %d, %.3f)", ref, i, delta);
            }
            opMode.sleep( sleepThreshold );
        }
        stop();
    }

}
