package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "UltimateGoal Autonomous Simple 2", group = "Linear Opmode")
public class UltimateGoalAutonomousSimple2 extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode()  {
        robot = new Robot(this);

        robot.init();

        robot.launcher.two(robot.launcher.AIMER_DOWN_POS);

        robot.launcher.holyHandGranadeManuel();

        robot.launcher.setScorpion_tail();

        robot.launcher.closeDingusKahn();

        waitForStart();

        robot.launcher.one();

        robot.launcher.two(0.413);

        robot.drivetrain.encoder_drive_mm(920, 0.35);

        robot.IMU.turn_to_heading(0.25, -118);

        robot.intake.setIntakePowers(-0.08);

        robot.drivetrain.encoder_drive_mm(1925, 0.3);

        robot.intake.salivate();

        robot.drivetrain.encoder_drive_mm(-300, 0.3);

        robot.IMU.turn_to_heading(0.25, -78);

        sleep(500);

        robot.launcher.five_ThreeSir_Three();

        sleep(2000);

        robot.IMU.turn_to_heading(0.25, -66);

        robot.launcher.setScorpion_tail(0);

        sleep(3500);

        robot.drivetrain.encoder_drive_mm(200, 0.4);


        robot.launcher.two(0);
    }
}