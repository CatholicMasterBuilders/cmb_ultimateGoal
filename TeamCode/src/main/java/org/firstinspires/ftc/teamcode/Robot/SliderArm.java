package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class SliderArm {

    // Data handles to ftc app
    Robot robot;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    LinearOpMode opMode;

    //Hardware Components
    DcMotor right_slider;
    DcMotor left_slider;
    TouchSensor slider_bottom_touch;

    //Variables
    int target_position = 0;
    int junk_pos = 0;
    int current_arm_index = 0;

    //Constants
    public final double GEAR_RATIO = 2;
    public final double COUNTS_PER_MOTOR_REV = 1497.3 * GEAR_RATIO;
    public final double COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV / 360;

    // Constants for height control
    public final double SLIDER_ARM_START_POSITION = 0;
    public final double SLIDER_ARM_STOP_POSITION = 6020;
    public final double SLIDER_ARM_LEVEL1_POSITION = SLIDER_ARM_START_POSITION;
    public final double SLIDER_ARM_LEVEL2_POSITION = 4;
    public final double SLIDER_ARM_LEVEL3_POSITION = 9;
    public final double SLIDER_ARM_LEVEL4_POSITION = 14;
    public final double SLIDER_ARM_LEVEL5_POSITION = 19;
    public final double SLIDER_ARM_THRESHOLD = 150;

    public class Slider_levels {
        double slider_level;
        public Slider_levels(double slider_position) {slider_level = slider_position;}
    }

    List<Slider_levels> slider_levels = new ArrayList<Slider_levels>();

    public void setSliderLevels () {
        slider_levels.add(new Slider_levels (SLIDER_ARM_LEVEL1_POSITION));
        slider_levels.add(new Slider_levels(SLIDER_ARM_LEVEL2_POSITION));
        slider_levels.add(new Slider_levels (SLIDER_ARM_LEVEL3_POSITION));
        slider_levels.add(new Slider_levels(SLIDER_ARM_LEVEL4_POSITION));
        slider_levels.add(new Slider_levels(SLIDER_ARM_LEVEL5_POSITION));
    }

    //Constructor
    public SliderArm (Robot robot) { setRobot(robot);}

    public void init () {
        hardwareMap = getRobot().hardwareMap;
        telemetry = getRobot().telemetry;
        opMode = getRobot().opMode;

        right_slider = hardwareMap.get(DcMotor.class, "right_slider");
        left_slider = hardwareMap.get(DcMotor.class, "left_slider");
        slider_bottom_touch = hardwareMap.get(TouchSensor.class, "slider_bottom_touch");

        setMode( DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode( DcMotor.RunMode.RUN_USING_ENCODER);

        right_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set left to reverse so the encoders count up from the bottom to the top
        left_slider.setDirection(DcMotorSimple.Direction.REVERSE);

        setPower(0);
        setTargetPosition(0);

        // Initialize the preset levels for the lift
        setSliderLevels();
    }
    public void setMode( DcMotor.RunMode mode ) {
        left_slider.setMode( mode );
        right_slider.setMode( mode );
    }

    public void setPower( double power ) {
        left_slider.setPower(power);
        right_slider.setPower(power);
    }
    private void setTargetPosition( double position ) {
        left_slider.setTargetPosition( (int) position );
        right_slider.setTargetPosition( (int) position );
    }

    // Is the slider arm currently closing
    public boolean isClosing( double power ) {
        // A positive power means the motors are trying to close the arms
        return power < 0;
    }

    // Is the slider arm currently opening
    public boolean isOpening( double power ) {
        return ! isClosing( power );
    }

    // Is the slider arm fully closed or getting close to it
    private boolean isSliderAtBottom( double current_position ) {
        return ( current_position <= SLIDER_ARM_START_POSITION ) || Math.abs( current_position - SLIDER_ARM_START_POSITION ) < SLIDER_ARM_THRESHOLD ||
                slider_bottom_touch.isPressed();
    }

    // Is the slider arm fully open or getting close to it
    private boolean isSliderAtTop( double current_position ) {
        return ( current_position >= SLIDER_ARM_STOP_POSITION ) || Math.abs( current_position - SLIDER_ARM_STOP_POSITION ) < SLIDER_ARM_THRESHOLD;
    }

    // Test using RUN_TO_POSITION for all movement
    public void junkTest( double power ) {
        if ( Math.abs(power-0.0) < 0.001 ) {
            // skip if no power
            return;
        }
        // The power input from gamepad is - for raising arms
        RobotLog.dd("CMB_RR", "pow: %f junk_pos: %d", power, junk_pos);
        power *= -1;
        junk_pos += 300.0 * power ;
        junk_pos = Range.clip( junk_pos, (int)SLIDER_ARM_START_POSITION, (int)SLIDER_ARM_STOP_POSITION );
        setTargetPosition( junk_pos );
        setMode( DcMotor.RunMode.RUN_TO_POSITION );
        RobotLog.dd("CMB_RR", "pow: %f junk_pos: %d", power, junk_pos);
        setPower( power );
    }

    // Set an analog power for the slide rails
    public void setSliderPower (double power) {
        // If the motors are currently running to a preset, only overide if there is joystick input
        if ( left_slider.getMode() == DcMotor.RunMode.RUN_TO_POSITION && Math.abs(power-0.0) < 0.001 ) {
//            RobotLog.dd("CMB_RR", "Skipping slider power because i feel like it");
            return;
        }

        setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        // The power input from gamepad is - for raising arms
        power *= -1;
        double current_postition = getSliderPosition();

        if ( !robot.opMode.gamepad2.dpad_up && // Hack to allow resetting sliding rails
             ( ( isSliderAtBottom( current_postition ) && isClosing( power ) ) ||
               ( isSliderAtTop( current_postition ) && isOpening( power ) )
             )
           )
        {
            // Stop the motors if we are near the top and opening or the bottom and closing
            stopSliders();
        }
        else {
            setPower( power );
        }
    }

    public void stopSliders () {
        setPower(0);
    }

    public double getSliderPosition() {
        int left_arm_position = left_slider.getCurrentPosition();
        int right_arm_position = right_slider.getCurrentPosition();

        int avgArmPosition = (left_arm_position + right_arm_position) / 2;

        return avgArmPosition;
    }

    public void toggleSliderLevels (boolean down) {
        if (down == true) {
            current_arm_index = 0;
            Slider_levels current_level = slider_levels.get(current_arm_index);
            setSliderPosition(current_level.slider_level);
        }
        else {
            toggleSliderLevels();
        }
    }
    public void toggleSliderLevels () {
        Slider_levels current_level = slider_levels.get(current_arm_index);

        setSliderPosition(current_level.slider_level);

        if (current_arm_index == 4){
            current_arm_index = 0;
        }
        else {
            current_arm_index ++;
        }
    }

    public double setSliderPosition (double inches) {

        target_position = (int) (inches * 285.71);

        stopSliders();

        setTargetPosition( target_position );
//        RobotLog.dd("CMB_RR", "setSliderPosition %d", target_position);

        setMode( DcMotor.RunMode.RUN_TO_POSITION );
        setPower(.8);

        return inches;

    }

    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }
}
