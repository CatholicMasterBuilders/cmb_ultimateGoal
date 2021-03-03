package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "UltimateGoal Autonomous Simple 3", group = "Linear Opmode")
public class UltimateGoalAutonomousSimple3 extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this);

        robot.init();
        robot.drivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.launcher.two(robot.launcher.AIMER_DOWN_POS);

        robot.launcher.holyHandGranadeManuel();

        robot.launcher.setScorpion_tail();

        robot.launcher.closeDingusKahn();

        waitForStart();

        robot.launcher.two(0.425);
        robot.launcher.one(0.7);


        robot.drivetrain.encoder_drive_mm(1800, 0.5);

        robot.placingSystem.placeGoal();

        robot.drivetrain.encoder_drive_mm(-481, 0.4);

        robot.placingSystem.retractArm();

        robot.IMU.turn(0.25, 8);


        robot.launcher.five_ThreeSir_Three();

        sleep(3000);

        robot.launcher.two(0.432);


        robot.IMU.turn(0.25, 3);

        robot.launcher.setScorpion_tail(0);

        sleep(3000);

        robot.launcher.two(0);

        sleep(3000);

        robot.launcher.pullHolyGranadePin();

        robot.intake.slurp();

        robot.IMU.turn(0.25,-10);

        robot.drivetrain.encoder_drive_mm(600,0.4);

        robot.drivetrain.encoder_drive_mm(-300, 0.3);



    }
}