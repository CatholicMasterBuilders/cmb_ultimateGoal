package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp (name = "EMUTest")
@Disabled
public class EMUTest extends LinearOpMode {
    Robot robot;

    public void runOpMode() {
        robot = new Robot(this);

        // Initialize the Drivetrain and EMU
        robot.drivetrain.drive_init();
        robot.emu.init();

        waitForStart();

        robot.emu.test();
    }
}
