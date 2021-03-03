package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
import org.firstinspires.ftc.teamcode.Navigation.EncoderTracker;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.DataTypes.Heading.toNormalizedHeading;

public class Drivetrain {
    public boolean debugStall = false;
    public class StallDetector {
        int leftstart;
        int leftcurrent;
        int leftprevious;
        int rightstart;
        int rightcurrent;
        int rightprevious;
        boolean hasStarted = false;
        boolean prevHasBeenStalled = false;

        final int stallthreshold = 0;
        final static int START_THRESHOLD = 100;

        private void init() {
            leftstart = left_front.getCurrentPosition();
            rightstart = right_front.getCurrentPosition();
        }
        private void update() {
            update( left_front.getCurrentPosition(), right_front.getCurrentPosition() );
        }
        private void update(int leftpostion, int rightposition) {
            leftprevious = leftcurrent;
            leftcurrent = leftpostion;
            rightprevious = rightcurrent;
            rightcurrent = rightposition;
            if ( ! hasStarted ) {
                if ( Math.abs(leftpostion-leftstart) > START_THRESHOLD && Math.abs(rightposition-rightstart) > START_THRESHOLD )
                    hasStarted = true;
            }
        }

        private boolean itstalled() {
            boolean iscurrentstall = false;
            boolean isstalled = false;
            boolean isleftstalled = false;
            boolean isrightstalled = false;
            if (debugStall)RobotLog.dd("CMBRR", "%d,%d,%d,%d", leftcurrent,leftprevious,rightcurrent,rightprevious);
            if ( hasStarted ) {
                if (Math.abs(leftcurrent - leftprevious) <= stallthreshold) {
                    isleftstalled = true;
                }
                if (Math.abs(rightcurrent - rightprevious) <= stallthreshold) {
                    isrightstalled = true;
                }
                iscurrentstall = isleftstalled || isrightstalled;
            }
            isstalled = prevHasBeenStalled && iscurrentstall;
            if ( isstalled )
                RobotLog.dd("CMBRR", "Start,curr,prev:%d,%d,%d,%d,%d,%d",
                        leftstart,leftcurrent,leftprevious,rightstart,rightcurrent,rightprevious);

            prevHasBeenStalled = iscurrentstall;
//            return isleftstalled || isrightstalled;
            return isstalled;
        }
    }

    Robot robot;
    LinearOpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public DcMotor left_front;
    public DcMotor right_front;
    public DcMotor left_rear;
    public DcMotor right_rear;
    public EncoderTracker encoderTracker;

    // Constants for using encoders         We switched to 40:1 on our Drivetrain
    static final double COUNTS_PER_MOTOR_REV = 145.6;    //It is 1680 for a 60:1 gear ratio
    static final double DRIVE_GEAR_REDUCTION = 80/24;     // This is < 1.0 if geared UP
    static final double MM_PER_INCH = 25.4;
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double WHEEL_DIAMETER_MILLIMETERS = WHEEL_DIAMETER_INCHES * MM_PER_INCH;
    static final double WHEEL_TRACK_INCHES = 17; // just a guess for track
    static final double DRIVE_WHEEL_TRACK_MILLIMETERS = WHEEL_TRACK_INCHES * MM_PER_INCH;
    static final double DRIVE_WHEEL_COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_WHEEL_COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MILLIMETERS * Math.PI);


    //start of track wheel-
    private static final double TRACKING_WHEEL_TICKS_REVOLUTION = 360*4;
    private static final double TRACKING_WHEEL_DIAMETER = 51; // Units in mm
    private static final double TRACKING_WHEEL_COUNTS_PER_MM =
            TRACKING_WHEEL_TICKS_REVOLUTION / (TRACKING_WHEEL_DIAMETER * Math.PI);
    private static final double TRACKING_WHEEL_COUNTS_PER_INCH =
            TRACKING_WHEEL_COUNTS_PER_MM * MM_PER_INCH;

    private static final double TRACKING_WHEEL_TRACK_MM = 155;
    private static final double TRACKING_WHEEL_TICK_DIFFERENTIAL_PER_ONE_DEGREE_OF_HEADING_DELTA =
            2 * Math.PI * TRACKING_WHEEL_TRACK_MM * TRACKING_WHEEL_COUNTS_PER_MM / 360;

    // Note that these are not final because they will change config when the encoder wheels
    // are added
    static double COUNTS_PER_INCH = DRIVE_WHEEL_COUNTS_PER_INCH;
    static double COUNTS_PER_MILLIMETER = DRIVE_WHEEL_COUNTS_PER_MM;
    static double WHEEL_TRACK_MILLIMETERS = DRIVE_WHEEL_TRACK_MILLIMETERS;

    //end of track wheel


    // Toggle between incremental position calculation or total position calculation
    public void disablePositionIntegrator() {
        robot_position_integrator =false;
//        robot_position_integrator =true;
    }

    public void enablePositionIntegrator() {
        robot_position_integrator =true;
    }

    boolean robot_position_integrator = false;
//    boolean robot_position_integrator = true;

    // Constructor
    public Drivetrain(Robot robot ) {
        setRobot(robot);
    }

    public void drive_init() {
        // store the hardwareMap and telemetry for easy usage
        hardwareMap = getRobot().hardwareMap;
        telemetry = getRobot().telemetry;
        opMode = getRobot().opMode;

        left_front = hardwareMap.get(DcMotor.class, "left_front");
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_rear = hardwareMap.get(DcMotor.class, "left_rear");
        right_rear = hardwareMap.get(DcMotor.class, "right_rear");

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        left_rear.setDirection(DcMotorSimple.Direction.REVERSE);
        right_rear.setDirection(DcMotorSimple.Direction.FORWARD);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Setup an encoder tracker which can be used with this drivetrain.
        encoderTracker = new EncoderTracker( left_rear, right_rear );

        // Setup the correct encoder constants
//        if ( robot.useEMUWheels) {
//            COUNTS_PER_INCH = TRACKING_WHEEL_COUNTS_PER_INCH;
//            COUNTS_PER_MILLIMETER = TRACKING_WHEEL_COUNTS_PER_MM;
//            WHEEL_TRACK_MILLIMETERS = TRACKING_WHEEL_TRACK_MM;
//        }

//        setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMode( DcMotor.RunMode.RUN_USING_ENCODER);
//        setMode( DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        RobotLog.dd("CMBRR", "TRACKING_WHEEL_TICKS_REVOLUTION:%f", TRACKING_WHEEL_TICKS_REVOLUTION);
        RobotLog.dd("CMBRR", "TRACKING_WHEEL_COUNTS_PER_MM:%f", TRACKING_WHEEL_COUNTS_PER_MM);
        RobotLog.dd("CMBRR", "TRACKING_WHEEL_TICKS_REVOLUTION:%f", TRACKING_WHEEL_TICKS_REVOLUTION);
        RobotLog.dd("CMBRR", "TRACKING_WHEEL_TICK_DIFFERENTIAL_PER_ONE_DEGREE_OF_HEADING_DELTA:%f", TRACKING_WHEEL_TICK_DIFFERENTIAL_PER_ONE_DEGREE_OF_HEADING_DELTA);
    }
    private DcMotorSimple.Direction getReverseDirection ( DcMotor motor ) {
        return motor.getDirection() == DcMotorSimple.Direction.REVERSE
                ? DcMotorSimple.Direction.FORWARD
                : DcMotorSimple.Direction.REVERSE;
    }
    public void reverseDrivetrain() {
        left_front.setDirection( getReverseDirection( left_front ) );
        right_front.setDirection( getReverseDirection( right_front ) );
        left_rear.setDirection( getReverseDirection( left_rear ) );
        right_rear.setDirection( getReverseDirection( right_rear ) );
    }

    public void teleOpControl (Gamepad g1, double maxPower) {
        exponentialControl( g1, maxPower );
    }

    public void linearControl (Gamepad g1, double maxPower) {
        double drive_power = g1.left_stick_y;
        double turn_power = g1.left_stick_x;

        // Decrease the maximum power for turns greater than 10%
        if (Math.abs(turn_power - 0) > 0.1) {
            maxPower = maxPower / 2;
        }

        double left_power = ((drive_power - turn_power) * maxPower);
        double right_power = ((drive_power + turn_power) * maxPower);
        RobotLog.dd("CMBRR", "power (drive,turn,left,right) (%f, %f,%f, %f)", drive_power,turn_power, left_power, right_power);


        robot.drivetrain.setPower(left_power, right_power);
    }

    private void exponentialControl (Gamepad g1, double maxPower) {
        // When calculating drive power input from the joystick, we have to make the input
        // positive if the exponent is undefined for negative numbers
        double drive_power = Math.pow(Math.abs(g1.left_stick_y), 2.5) * Math.signum ( g1.left_stick_y );
        double turn_power = g1.left_stick_x;

        // Decrease the maximum power for turns greater than 10%
        if (Math.abs(turn_power) > 0.1) {
            maxPower = maxPower * 0.8;
        }

        double left_power = Range.clip((drive_power - turn_power) * maxPower, -1.0, 1.0);
        double right_power = Range.clip((drive_power + turn_power) * maxPower, -1.0, 1.0);

        setPower(left_power, right_power);
    }

    public void setMode( DcMotor.RunMode runMode ) {
        left_front.setMode(runMode);
        right_front.setMode(runMode);
        left_rear.setMode(runMode);
        right_rear.setMode(runMode);
    }

    public void setPower( double left_fr_power, double left_bk_power, double right__fr_power, double right__bk_power ) {
        left_front.setPower(left_fr_power);
        right_rear.setPower(right__bk_power);
        right_front.setPower(right__fr_power);
        left_rear.setPower(left_bk_power);
    }
    public void setPower( double left_power, double right_power ) {
//        robot.IMU.imu.startAccelerationIntegration(robot.position, robot.velocity, 10);
        left_front.setPower(left_power);
        right_rear.setPower(right_power);
        right_front.setPower(right_power);
        left_rear.setPower(left_power);
    }

    public void stopPower(){
        RobotLog.dd("CMBRR", "stop power");

        left_front.setPower(0);
        right_rear.setPower(0);
        right_front.setPower(0);
        left_rear.setPower(0);
//        robot.IMU.imu.stopAccelerationIntegration();
    }

    public void setTargetPosition( int left_target, int right_target ) {
        left_front.setTargetPosition(left_target);
        left_rear.setTargetPosition(left_target);
        right_front.setTargetPosition(right_target);
        right_rear.setTargetPosition(right_target);
    }

    public void driveLoop (double Power){
    }

    public void encoder_drive_inches (double inches, double power) {
        double encoder_distance = inches * COUNTS_PER_INCH;
        encoder_drive(encoder_distance, encoder_distance, power, power);
    }
    public void encoder_drive_mm (double millimeters, double power) {
        double encoder_distance = millimeters * COUNTS_PER_MILLIMETER;
        encoder_drive(encoder_distance, encoder_distance, power, power);
    }

    private void encoder_drive (double encoder_distance, double power) {
        encoder_drive(encoder_distance, encoder_distance, power, power);
    }

    public void encoder_drive_inches(double left_inches, double right_inches, double left_power, double right_power) {
        encoder_drive( left_inches * COUNTS_PER_INCH,
                right_inches * COUNTS_PER_INCH,
                left_power, right_power );
    }
    public void encoder_drive_mm(double left_mm, double right_mm, double left_power, double right_power) {
        encoder_drive( left_mm * COUNTS_PER_MILLIMETER,
                right_mm * COUNTS_PER_MILLIMETER,
                left_power, right_power );
    }

    private void encoder_drive(double left_enc_dist, double right_enc_dist, double left_power, double right_power) {

//        setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMode( DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.dd("CMBRRRR", "drive start (x,y) = (%d, %d)", robot.robot_coord.x, robot.robot_coord.y);

        int new_left_target;
        int new_right_target;

        int startLeft = left_rear.getCurrentPosition();
        int startRight = right_rear.getCurrentPosition();

        new_left_target = startLeft - (int) (left_enc_dist);
        new_right_target = startRight - (int) (right_enc_dist);

        setTargetPosition(new_left_target,new_right_target);
        setMode( DcMotor.RunMode.RUN_TO_POSITION);

        double last_heading = robot.IMU.get_x_axis_heading();

        StallDetector stallDetector = new StallDetector();
        stallDetector.init();
        encoderTracker.init();
        boolean shouldThrottleUpdates = true;

        setPower(left_power,right_power);
        while (opMode.opModeIsActive() && left_rear.getPower() != 0.0 && right_rear.getPower() != 0.0) {
            stallDetector.update();

            if (opMode.opModeIsActive() && (left_rear.isBusy() == false && right_rear.isBusy() == false) || stallDetector.itstalled() ) {
                stopPower();
                shouldThrottleUpdates = false;

                if ( stallDetector.itstalled() ){
                    RobotLog.dd("CMBRR", "encoder_drive: stalled");
                }

                if ( robot_position_integrator == false ) { // only update once
//                    encoderTracker.update();
                    robot.updateRobotPosition(robot.getCurrentCoordinate(), last_heading, shouldThrottleUpdates);
                }
            }

            if ( robot_position_integrator == true ) { // update every iteration
//                encoderTracker.update();
                robot.updateRobotPosition(robot.getCurrentCoordinate(), last_heading, shouldThrottleUpdates);
            }
        }

        int endLeft = left_front.getCurrentPosition();
        int endRight = right_front.getCurrentPosition();

        RobotLog.dd("CMBRRR", "encoder_drive_millimeters: (desired,actual_wheel,actual_track_wheel) = (%d,%d,%d) millimeters",
                (int)(left_enc_dist/COUNTS_PER_MILLIMETER),
                (int)( ((endLeft-startLeft)+(endRight-startRight))/2 / COUNTS_PER_MILLIMETER ),
                getTotalDistanceTravelled());

        RobotLog.dd("CMBRR", "encoder_turn: twticks delta(left,right) (%d, %d) actual",
                encoderTracker.getLeftEncoderDelta(EncoderTracker.DeltaType.TOTAL),
                encoderTracker.getRightEncoderDelta(EncoderTracker.DeltaType.TOTAL));

        RobotLog.dd("CMBRRRR", "drive end (x,y) = (%d, %d)", robot.robot_coord.x, robot.robot_coord.y);

    }

//    private void encoder_turn(double left_mm, double right_mm, double left_power, double right_power) {
    private void encoder_turn(double degrees, double left_power, double right_power) {
//        setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMode( DcMotor.RunMode.RUN_USING_ENCODER);
        RobotLog.dd ("CMBRRRR", "turn start (x,y) = (%d,%d)", robot.robot_coord.x, robot.robot_coord.y);

        double millimeter_differential = Math.abs( ( TRACKING_WHEEL_TRACK_MM * 2 * Math.PI ) * (degrees / 360) );

        int target_differential = (int)(millimeter_differential * TRACKING_WHEEL_COUNTS_PER_MM);
        int new_left_target;
        int new_right_target;
        int leftstart = encoderTracker.getLeftCurrentPosition();
        int rightstart = encoderTracker.getRightCurrentPosition();
        int leftcurrent, rightcurrent;

        RobotLog.dd("CMBRR", "encoder_turn: start (left,right,differential) (%d, %d, %d) twticks", leftstart, rightstart, target_differential);

//        setTargetPosition(new_left_target,new_right_target);
//        setMode( DcMotor.RunMode.RUN_TO_POSITION);
        setMode( DcMotor.RunMode.RUN_USING_ENCODER);

        double last_heading = robot.IMU.get_x_axis_heading();

        StallDetector stallDetector = new StallDetector();
        stallDetector.init();
        encoderTracker.init();
        boolean shouldThrottleUpdates = true;

        setPower(left_power,right_power);
        int left_delta, right_delta, prev_differential = 0, curr_differential = 0;
        while (opMode.opModeIsActive()
                && left_front.getPower() != 0.0
                && right_front.getPower() != 0.0) {
            stallDetector.update();

            leftcurrent = encoderTracker.getLeftCurrentPosition();
            rightcurrent = encoderTracker.getRightCurrentPosition();
            left_delta = (leftcurrent - leftstart);
            right_delta = (rightcurrent - rightstart);
            curr_differential = target_differential - Math.abs((left_delta-right_delta));
            if ( prev_differential == 0 ) { prev_differential = curr_differential; }
//            RobotLog.dd("CMBRR", "encoder_turn: ticks (left,right,differential) (%d, %d, %d, %d) twticks", leftcurrent, rightcurrent, left_delta, right_delta);

//            RobotLog.dd("CMBRR", "encoder_turn: drive (prev, curr) (%d, %d)",
//                    prev_differential, curr_differential);
            if (opMode.opModeIsActive() &&
                    ( Math.abs(curr_differential) <= 50 ) || (Math.signum(prev_differential) != Math.signum(curr_differential)) ||
                    stallDetector.itstalled() ) {
                stopPower();
                shouldThrottleUpdates = false;

                if ( stallDetector.itstalled() ){
                    RobotLog.dd("CMBRR", "encoder_turn: stalled");
                }

                if ( robot_position_integrator == false ) { // only update once
                    encoderTracker.update();
                    robot.updateRobotPosition(robot.getCurrentCoordinate(), last_heading, shouldThrottleUpdates);
                }
            }

            if ( robot_position_integrator == true ) { // update every iteration
//                encoderTracker.update();
                robot.updateRobotPosition(robot.getCurrentCoordinate(), last_heading, shouldThrottleUpdates);
            }
        }
        RobotLog.dd("CMBRR", "encoder_turn: twticks  delta(left,right) (%d, %d) actual", encoderTracker.getLeftEncoderDelta(EncoderTracker.DeltaType.TOTAL),
                encoderTracker.getRightEncoderDelta(EncoderTracker.DeltaType.TOTAL));
        RobotLog.dd ("CMBRRRR", "turn end (x,y) = (%d,%d)", robot.robot_coord.x, robot.robot_coord.y);
    }

    public void turn (double degree_turn, double turn_power){
        double start_heading = robot.IMU.get_x_axis_heading();
        //This is all in millimeters.
        // The difference in millimeters driven between wheels for 1 full 360 degree turn is 2*pi*WHEEL_TRACK
        // I think this formula is if we are measuring the difference using rotations
//        double turn_millimeters = Math.abs( ( WHEEL_TRACK_MILLIMETERS * 2 * Math.PI ) * (degree_turn / 360) ) / 2; // divide by two because each wheel drives half the difference
//        double turn_millimeters = Math.abs( ( TRACKING_WHEEL_TRACK_MM * 2 * Math.PI ) * (degree_turn / 360) ) / 2; // divide by two because each wheel drives half the difference
//        double left_millimeters = turn_millimeters;
//        double right_millimeters = -turn_millimeters;

        double left_power = turn_power;
        double right_power = turn_power;
        if (degree_turn > 0){
//            left_millimeters *= -1;
//            right_millimeters *= -1;
            right_power *= -1;
        }
        else {
            left_power *= -1;
        }

//        encoder_drive_mm(left_millimeters, right_millimeters, turn_power, turn_power );
//        encoder_turn(left_millimeters, right_millimeters, left_power, right_power );
        encoder_turn(degree_turn, left_power, right_power );

        double end_heading = robot.IMU.get_x_axis_heading();
        double degrees_turned = toNormalizedHeading( end_heading - start_heading );
        RobotLog.dd ("CMBRRR", "turn %f degrees, %f degrees turned, end heading %f", degree_turn, degrees_turned, end_heading);
//        setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        setMode( DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void turn_to_heading( double turn_speed, double heading ) {
        double turn_distance = robot.IMU.getHeadingDeltaFromCurrent( heading );
        RobotLog.dd(robot.DBG_TAG, "dt::turn_to_heading: desired turnDistance = %f", turn_distance);

        turn( turn_distance, turn_speed);
    }

    public double getHeadingDelta () {
        return encoderTracker.getEncoderHeadingDelta( EncoderTracker.DeltaType.TOTAL );
    }

    public int getTotalDistanceTravelled() {
        return  (int) ( encoderTracker.getAverage( EncoderTracker.DeltaType.TOTAL ) / TRACKING_WHEEL_COUNTS_PER_MM );
    }

    public double getDistanceTravelled() {
        return   ( encoderTracker.getAverage( EncoderTracker.DeltaType.INCREMENTAL ) / TRACKING_WHEEL_COUNTS_PER_MM );
    }

    // Getters and Setters
    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    // **************************************************************************************
    // Testing functions
    // **************************************************************************************
    public void test() {
        telemetry.setAutoClear( false );
        telemetry.addData("Push x to test Motor:", "left front");
        telemetry.update();
        while (opMode.opModeIsActive()) {
            if ( opMode.gamepad1.x ) {
                testDriveMotor(robot.drivetrain.left_front);
                break;
            }
        }

//        telemetry.addData("Push x to test Motor:", "left rear");
//        telemetry.update();
//        while (opMode.opModeIsActive()) {
//            if ( opMode.gamepad1.x ) {
//                testDriveMotor(robot.drivetrain.left_rear);
//                break;
//            }
//        }

        telemetry.addData("Push x to test Motor:", "right front");
        telemetry.update();
        while (opMode.opModeIsActive()) {
            if ( opMode.gamepad1.x ) {
                testDriveMotor(robot.drivetrain.right_front);
                break;
            }
        }

//        telemetry.addData("Push x to test Motor:", "right rear");
//        telemetry.update();
//        while (opMode.opModeIsActive()) {
//            if ( opMode.gamepad1.x ) {
//                testDriveMotor(robot.drivetrain.right_rear);
//                break;
//            }
//        }

        // Test driving forward one tile and back one tile to see if the encoder calcs are correct
        telemetry.addLine("Push x to drive forward 1 tile");
        telemetry.update();
        while (opMode.opModeIsActive()) {
            if ( opMode.gamepad1.x ) {
                encoder_drive_inches(24, 0.25);
                break;
            }
        }
        // Test driving forward one tile and back one tile to see if the encoder calcs are correct
        telemetry.addLine("Push x to drive backaward 1 tile");
        telemetry.update();
        while (opMode.opModeIsActive()) {
            if ( opMode.gamepad1.x ) {
                encoder_drive_inches(-24, 0.25);
                break;
            }
        }

    }
    private void testDriveMotor(DcMotor motor) {
//        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Test going forward
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("RUN_USING_ENCODER forward starting endcoder: ", motor.getCurrentPosition());
        telemetry.update();
        motor.setPower(0.1);
        opMode.sleep(1000);
        telemetry.addData("speeding up endcoder: ", motor.getCurrentPosition());
        telemetry.update();
        motor.setPower(0.5);
        opMode.sleep(1000);
        telemetry.addData("Going full throttle endcoder: ", motor.getCurrentPosition());
        telemetry.update();
        motor.setPower(1);
        opMode.sleep(1000);
        motor.setPower(0);
        telemetry.addData("Done endcoder: ", motor.getCurrentPosition());
        opMode.sleep(100);

        //Test going backward
        telemetry.addData("RUN_USING_ENCODER backward starting endcoder: ", motor.getCurrentPosition());
        telemetry.update();
        motor.setPower(-0.1);
        opMode.sleep(1000);
        telemetry.addData("speeding up endcoder: ", motor.getCurrentPosition());
        telemetry.update();
        motor.setPower(-0.5);
        opMode.sleep(1000);
        telemetry.addData("Going full throttle endcoder: ", motor.getCurrentPosition());
        telemetry.update();
        motor.setPower(-1);
        opMode.sleep(1000);
        motor.setPower(0);
        telemetry.addData("Done endcoder: ", motor.getCurrentPosition());
        opMode.sleep(100);
    }
}