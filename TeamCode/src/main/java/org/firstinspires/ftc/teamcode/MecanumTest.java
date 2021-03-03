package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class MecanumTest extends LinearOpMode {

    Robot robot;
    Drivetrain drivetrain;

    @Override
    public void runOpMode(){
        robot.drivetrain.drive_init();

        waitForStart();
        while (opModeIsActive()) {
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

//            drivetrain.left_front.setPower(v1); //front
//            drivetrain.right_front.setPower(v2); //front
//            drivetrain.left_rear.setPower(v3); //rear
//            drivetrain.right_rear.setPower(v4); //rear
        }
    }

    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }
}
