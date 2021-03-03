package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Navigation.EMU;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import static org.firstinspires.ftc.teamcode.DataTypes.Heading.toNormalizedHeading;

/**
 * Calibrate the EMU
 */
@TeleOp(name = "EMU Calibration Test", group = "Sensor")
@Disabled                            // Uncomment this to add to the opmode list
public class EMUCalibrationTest extends LinearOpMode
{
    Robot robot;
    EMU emu;
    int left_ticks, right_ticks;

    @Override public void runOpMode() {
        // Create and initialize the components for this test
        robot = new Robot(this);
        robot.drivetrain.drive_init();
        robot.IMU.IMU_calibration();

        // Get the EMU TrackingWheels
        emu = new EMU( robot );
        emu.init();

        waitForStart();

        telemetry.addLine("Starting test:");

        // Perform a robot turn and capture the data from the EMU encoders and the IMU gyro
        // These two pieces of info can be used to calculate the track width for the robot
        // TODO: there is a reset of the encoders in IMU.turn that will have to be commented out
        // to make this work.
        int leftstart = emu.getLeftEncoderValue();
        int rightstart = emu.getRightEncoderValue();
        double start_headinng = robot.IMU.get_raw_heading();
        robot.IMU.turn(0.4, 90 );
        sleep(1000); // make sure the imu has settled
        double end_heading = robot.IMU.get_raw_heading();
        double degrees_turned = toNormalizedHeading(end_heading - start_headinng);
        int left_change = Math.abs( emu.getLeftEncoderValue() - leftstart );
        int right_change = Math.abs( emu.getRightEncoderValue() - rightstart );
        int encoder_difference = left_change + right_change;
        double track_width = emu.calculateTrackWidth( encoder_difference, degrees_turned );

        // Dump out the test results
//        telemetry.addData("Enc -", "left: %d right: %d", emu.getLeftEncoderValue(), emu.getRightEncoderValue());
        telemetry.addData("Enc -", "start: (%d,%d) end: (%d,%d) delta: (%d,%d)", leftstart, rightstart, emu.getLeftEncoderValue(),emu.getRightEncoderValue(), left_change, right_change);
        telemetry.addData("Heading - ", "S:%.3f E:%.3f Delta:%.3f", start_headinng, end_heading, degrees_turned);
        telemetry.addData("Track:", track_width);

        // Wait around for the user to record the data
        while (opModeIsActive()) {
            telemetry.update();
            idle();
        }
    }
}