package org.firstinspires.ftc.teamcode.Robot;

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
import org.firstinspires.ftc.teamcode.NaiveAccelerationIntegrator;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.DataTypes.Heading.toCompassHeading;
import static org.firstinspires.ftc.teamcode.DataTypes.Heading.toNormalizedHeading;

public class IMU {

    Robot robot;
    public BNO055IMU imu;
    Orientation angles;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    LinearOpMode opMode;

    public IMU(Robot robot) {
        setRobot(robot);
    }

    public void IMU_calibration (){
        opMode = getRobot().opMode;
        hardwareMap = getRobot().hardwareMap;
        telemetry = getRobot().telemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "RoverRuckusIMUCalibration.json";
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.accelerationIntegrationAlgorithm = new NaiveAccelerationIntegrator();
        imu.initialize(parameters);

//        // might consider this to reset the imu
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
//        opMode.sleep(100 );
//        // Force reset using the trigger register by setting the RST_SYS bit ( page 51 spec )
//        imu.write8(BNO055IMU.Register.SYS_TRIGGER,0x20);
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
//        opMode.sleep(100 );
    }

    /** Turn using the IMU to monitor the progress of the turn.  The IMU will make use of
     *  the drivetrain to actually perform the turn
     * @param turn_speed - Maximum speed for the turn
     * @param degree_turn - How many degrees to turn ( CCW is positive )
     */
    public void turn (double turn_speed, double degree_turn){
        double start_angle = get_heading();
        double start_heading = robot.IMU.get_compass_heading();
        double requested_degrees = degree_turn / 1.4; // reasonable guess based on TurnTest
        RobotLog.dd( robot.DBG_TAG, "Start turn %f degrees using speed of %f starting heading = %f\n", degree_turn, turn_speed, start_angle );
        if (degree_turn < 90) {
            requested_degrees = degree_turn / 1.6; // reasonable guess based on TurnTest
        }
        else  {
            requested_degrees = degree_turn / 1.4; // reasonable guess based on TurnTest
        }
        // Adjust turn speed and degree_turn based on turn direction.
        if ( requested_degrees < 0 ) {
            // This turn is CW so we have to flip the turn speed to spin the wheels in the right direction
            turn_speed *= -1;
            requested_degrees *= -1;
        }
        robot.drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.drivetrain.setPower( turn_speed, -turn_speed );

        int iteraction_cnt = 0;
        double elapsed_time = 0;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opMode.opModeIsActive() ) {
            iteraction_cnt++;
             if ( getHeadingDelta( start_heading ) >= requested_degrees ){
                robot.drivetrain.stopPower();
                elapsed_time = timer.milliseconds();
                // TODO: Run a test without these two lines and remove them if everything is ok
//                robot.drivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            }
        }
        RobotLog.dd( robot.DBG_TAG, "End turn %f degrees from %f to %f ( delta = %f ) using %d iterations in %f ms ( %f ms/loop ) \n",
                degree_turn, start_angle, get_heading(), getHeadingDelta( start_heading ), iteraction_cnt, elapsed_time, elapsed_time/iteraction_cnt );
    }

    /**
     * This method will align  the robot to the heading given.  The heading is field relative and
     * must be in the range [-179,180]
     *
     * @param turn_speed
     * @param heading
     */
    public void turn_to_heading( double turn_speed, double heading ) {
        double start_heading = get_x_axis_heading();
        double turn_distance = getHeadingDeltaFromCurrent( heading );
        RobotLog.dd(robot.DBG_TAG, "turn_to_heading: desired turnDistance = %f", turn_distance);

        // Adjust turn speed and degree_turn based on turn direction.
        if ( turn_distance < 0 ) {
            // This turn is CW so we have to flip the turn speed to spin the wheels in the right direction
            turn_speed *= -1;
            turn_distance *= -1;
        }
        robot.drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.drivetrain.setPower( turn_speed, -turn_speed );

        int iteraction_cnt = 0;
        double elapsed_time = 0;
        double prev_heading;
        double curr_heading = get_x_axis_heading();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opMode.opModeIsActive() ) {
            iteraction_cnt++;
            prev_heading = curr_heading;
            curr_heading = get_x_axis_heading( true );
            if ( Math.abs( toNormalizedHeading( heading - curr_heading ) ) // delta from target
                    <= 5 + 2 * Math.abs( toNormalizedHeading( prev_heading - curr_heading ) )) // threshold + delta from last reading
            {
                // Turn motors ReturnToThree outside the loop so they are always ReturnToThree even when stop is pressed
                break;
            }
        }
        robot.drivetrain.stopPower();
        elapsed_time = timer.milliseconds();
        // TODO: Run a test without these two lines and remove them if everything is ok
        robot.drivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double end_heading = get_x_axis_heading();
        RobotLog.dd( robot.DBG_TAG, "turn_to_heading: End turn %3.0f degrees from %3.0f to %3.0f ( delta = %3.0f, error %3.0f ) using %d iterations in %f ms ( %f ms/loop ) \n",
                turn_distance, start_heading, end_heading,
                toNormalizedHeading( start_heading - end_heading ),
                toNormalizedHeading( end_heading - heading ),
                iteraction_cnt, elapsed_time, elapsed_time/iteraction_cnt );
    }

    /**
     * This computes the delta between start_heading and the current position while accounting for
     * rotations around 0 degrees and 180 degrees
     * @param start_heading
     * @return the heading delta
     */
    public double getHeadingDelta( double start_heading ) {
        double degrees_turned = Math.abs(start_heading - robot.IMU.get_compass_heading()); // This works if the angle is relative
        degrees_turned -= 360*Math.round(degrees_turned/360); // This is also needed if the angle is absolute
        return Math.abs(degrees_turned);
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
            RobotLog.dd(robot.DBG_TAG, "getHeadingDeltaFromCurrent: invalid heading %f\n", final_heading);
            final_heading = toNormalizedHeading(final_heading);
        }

        double start_heading = get_x_axis_heading();
        double turn_distance = toNormalizedHeading( final_heading - start_heading );
        return turn_distance;
    }

    /**
     * Get the raw heading value from the IMU.  This is guaranteed to be in the range [-180,180]
     *      * but it is NOT relative to the field coordinate axes
     * @return raw IMU heading
     */
    public double get_heading (){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Normalize the heading into the range [-180,180)
        // TODO: This probably isn't needed because it seems like the imu already return normalized angles
        return toNormalizedHeading( angles.firstAngle );
    }

    /** returns the raw heading value from the IMU.  This is guaranteed to be in the range [0,360]
     * but it is NOT relative to the field coordinate axes
     * */
    public double get_compass_heading (){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // The imu gives values in the range of (-179,180] but we want the range to be [0,360]
        return toCompassHeading( angles.firstAngle );
    }

    /** returns the raw heading value from the IMU.  This is guaranteed to be in the range [0,360]
     * but it is NOT relative to the field coordinate axes
     * */
    public double get_raw_heading (){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    /**
     * Get the field relative heading.  x axis is 0, heading
     * increases in CCW direction and range is [-179,180]
     * @return
     */
    public double get_x_axis_heading (boolean debug){
        double real_heading = get_heading();
        double x_axis_heading = real_heading + robot.heading_adjustment;
        x_axis_heading = toNormalizedHeading(x_axis_heading);
        if ( debug ) RobotLog.dd("CMBRR", "IMU:get_x_axis_heading: raw heading %f, x_axis_heading %f", real_heading, x_axis_heading);

        return x_axis_heading;
    }
    public double get_x_axis_heading (){
        return get_x_axis_heading(true);
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    // Getters and Setters
    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }
}
