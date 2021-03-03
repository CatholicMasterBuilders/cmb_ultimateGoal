package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp (name = "DrivetrainTest")
@Disabled
public class DrivetrainTest extends LinearOpMode {
    Robot robot;

    public void runOpMode() {
        robot = new Robot(this);

        // Initialize the Drivetrain
//        robot.drivetrain.drive_init();
        robot.init();
        waitForStart();

        robot.drivetrain.test();
    }

}
