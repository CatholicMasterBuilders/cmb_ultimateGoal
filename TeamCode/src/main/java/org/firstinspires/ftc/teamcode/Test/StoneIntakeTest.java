package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="StoneIntakeTest", group= "Linear Opmode")
//@Disabled
public class StoneIntakeTest extends LinearOpMode {
    Robot robot;

    public void runOpMode() {
        robot = new Robot(this);

        robot.init();
        robot.drivetrain.reverseDrivetrain(); // So that the intake is forward

        waitForStart();

        double scale_factor = .000001;
        double speed = 0;
        double intake_speed = 0;
        while ( opModeIsActive() ) {
            speed += -gamepad1.left_stick_y * scale_factor;
            speed = Range.clip(speed, -1.0, 1.0);

            intake_speed += -gamepad1.right_stick_y * scale_factor;
            intake_speed = Range.clip(intake_speed, -1.0, 1.0);

            telemetry.addLine( "Left Joystick increases speed of motion");
            telemetry.addLine( "Right Joystick increases speed of wheels");
            telemetry.addLine( "x starts intake attempt");
            telemetry.addData("Speed: ", "%.3f", speed );
            telemetry.addData("Intake: ", "%.3f", intake_speed );
            telemetry.update();

            if ( gamepad1.x) {
                // Execute one test run
                robot.intake.setIntakePowers(-intake_speed);
                robot.drivetrain.encoder_drive_inches(24, speed);
                robot.drivetrain.encoder_drive_inches(3, .25);
                robot.drivetrain.encoder_drive_inches(-27, speed);
                robot.intake.salivate();
            }
        }
    }
}
