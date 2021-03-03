package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "UltimateGoal Autonomous Simple", group = "Linear Opmode")
public class UltimateGoalAutonomousSimple extends LinearOpMode {
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

        robot.launcher.two();
        robot.launcher.one();

        robot.IMU.turn_to_heading(0.25, 10);

        robot.drivetrain.encoder_drive_mm(2350, 0.25);

        sleep(500);

        robot.drivetrain.encoder_drive_mm(-800, 0.25);

        robot.IMU.turn_to_heading(0.25, 0);

        sleep(1000);

        robot.launcher.five_ThreeSir_Three();

        sleep(2000);

        robot.launcher.setScorpion_tail(0);

        sleep(3000);

        robot.drivetrain.encoder_drive_mm(250,0.3);

        robot.launcher.two(0);
    }
}
