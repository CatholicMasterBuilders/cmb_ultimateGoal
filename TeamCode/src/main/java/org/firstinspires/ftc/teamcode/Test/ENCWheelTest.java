package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name = "ENCWheelTest")
public class ENCWheelTest extends LinearOpMode{
        Robot robot;

        public void runOpMode() {
            robot = new Robot(this);

            // Initialize the Drivetrain
//        robot.drivetrain.drive_init();
            robot.init();
            waitForStart();
            RobotLog.dd("enc", "Start");
            while (opModeIsActive()){
                RobotLog.dd("CMBRR", "right_tracker: %d left_tracker %d left_motor %d right_motor %d",
                        robot.drivetrain.right_rear.getCurrentPosition(),
                        robot.drivetrain.left_rear.getCurrentPosition(),
                        robot.drivetrain.left_front.getCurrentPosition(),
                        robot.drivetrain.right_front.getCurrentPosition());
                sleep(50);
            }
            RobotLog.dd("enc", "Stop");
        }

    }

