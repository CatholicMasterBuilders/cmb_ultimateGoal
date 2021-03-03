package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp (name = "MecanumDriveTest")
@Disabled
public class MecanumDriveTest extends LinearOpMode {
    Robot robot;

    public void runOpMode() {
        robot = new Robot(this);
        robot.init();

        waitForStart();

        while (opModeIsActive()) {

            // Headless Driving
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = robot.IMU.get_heading();
            double joystickAngle = robotAngle - (Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4);
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(joystickAngle) + rightX;
            final double v2 = r * Math.sin(joystickAngle) - rightX;
            final double v3 = r * Math.sin(joystickAngle) + rightX;
            final double v4 = r * Math.cos(joystickAngle) - rightX;

            robot.drivetrain.setPower(v1, v3, v2, v4);

            telemetry.addData("stick_y", gamepad1.right_stick_y);
            telemetry.addData("LeftFrontMotorPower", v1);
            telemetry.addData("LeftRearMotorPower", v3);
            telemetry.addData("RightFrontMotorPower", v2);
            telemetry.addData("RightRearMotorPower", v4);
            telemetry.addData("gamepad Joy-Stick x", gamepad1.left_stick_y);
            telemetry.addData("gamepad Joy-Stick y", gamepad1.left_stick_x);

            telemetry.update();
        }
    }
}
