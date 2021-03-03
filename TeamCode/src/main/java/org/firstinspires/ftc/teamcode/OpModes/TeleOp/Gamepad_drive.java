package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.Robot;

//import org.opencv.core.Range;

@TeleOp (name = "Gamepad_drive")
public class Gamepad_drive extends LinearOpMode {
    Robot robot;
    boolean leftBumperPressed = false;
    boolean aPressed = false;
    boolean xPressed = false;
    boolean isPowerShots = false;
    boolean isChongusOff = true;
    boolean isScorpion_tailExtended = false;

    //TODO: Try to set the prev_gamepad1 to null to know that it is ready to receive a button.
    Gamepad prev_gamepad1 = new Gamepad();
    Gamepad prev_gamepad2 = new Gamepad();


    public void runOpMode() {
        robot = new Robot(this);
        robot.init();
        boolean isAiming = false;
        robot.launcher.holyHandGranadeManuel();
        robot.launcher.two(0);
        robot.launcher.setScorpion_tail();

        double maxPower;

        waitForStart();
        int connection_test = 1;
        int display_number = 0;
        while (opModeIsActive()) {
            if (connection_test++ % 1 == 0) {
                display_number++;
            }



            //**************************************
            // Controller 1:
            //**************************************
            maxPower = 0.8; // Set default maximum power
            if (gamepad1.left_bumper) {
                maxPower = maxPower / 2;
                robot.drivetrain.linearControl(gamepad1, maxPower);
            }
            else if (gamepad1.right_bumper) {
                maxPower = maxPower / 4;
                robot.drivetrain.linearControl(gamepad1, maxPower);
            }

            else {
                robot.drivetrain.teleOpControl(gamepad1, maxPower);
            }


            if (gamepad1.a){
                aPressed = true;
            }

            if (gamepad1.x){
                xPressed = true;
            }

            if (!gamepad1.a && aPressed){
                //If chongus is off, turn it on
                if(isChongusOff) {
                    robot.launcher.one(0.85);
                    isChongusOff = false;
                }

                //If chongus is on, turn it off.
                else if (!isChongusOff){
                    robot.launcher.holyHandGranadeManuel();
                    isChongusOff = true;
                }
                aPressed = false;
            }

            //Setter
            if (gamepad1.b){
                robot.launcher.five_ThreeSir_Three();
            }

            else {
                robot.launcher.hitFourYouHaveGoneTooFar();
            }

            //Aimer has three set positions
            if (gamepad1.dpad_up){
                robot.launcher.two(0.445);
            }

            else if (gamepad1.dpad_down){
                robot.launcher.two(0.03);
            }

            else if (gamepad1.dpad_left || gamepad1.dpad_right)
                robot.launcher.two(0.435);

            if (gamepad1.right_trigger > 0.1) {
                robot.intake.slurp();
                robot.launcher.pullHolyGranadePin();
                robot.launcher.one(-0.5);
            }

            else if (gamepad1.left_trigger > 0.1){
                robot.intake.spit();
            }

            else {
                robot.intake.salivate();
                if (isChongusOff)
                    robot.launcher.holyHandGranadeManuel();
            }

            if (gamepad1.y) {
                robot.launcher.setScorpion_tail(0);

            }

            else {
                robot.launcher.setScorpion_tail();
            }

            if (xPressed && !gamepad1.x) {
                robot.placingSystem.toggleDingusKahn();
                xPressed = false;
            }

            if (gamepad1.right_stick_y == 0){
                robot.placingSystem.thingy.setPower(-0.2);
            }

            else
                robot.placingSystem.setArmPower(gamepad1.right_stick_y);


            if (gamepad1.start) {
                isPowerShots = true;
            }

            if (isPowerShots) {
                robot.launcher.two(0.429);
                robot.launcher.one();

                robot.drivetrain.encoder_drive_mm(1319, 0.4);

                sleep(500);


                robot.IMU.turn(0.25, 17);

                sleep(3800);

                robot.launcher.five_ThreeSir_Three();

                robot.launcher.setScorpion_tail(0);

                robot.launcher.two(0.423);

                sleep(500);

                robot.IMU.turn(0.25, 6);

                robot.launcher.setScorpion_tail(0);

                sleep(2200);

                isPowerShots = false;
            }


            //Controller 2








            //TODO: Add a function to switch the state of isAiming with the a button
            //**************************************
            // Controller 2:
            //**************************************


            //**************************************
            // Shared Controls:
            //**************************************


            // Telemetry feedback
            telemetry.addData("left front", robot.drivetrain.left_front.getPower());
            telemetry.addData("right front", robot.drivetrain.right_front.getPower());
            telemetry.addData("right front Mode", robot.drivetrain.right_front.getCurrentPosition());
            telemetry.addData("DisplayNumber", display_number);
            telemetry.addData("Heading", robot.IMU.get_raw_heading());
            telemetry.addData("stick_y", gamepad1.right_stick_y);
            telemetry.addData("gamepad Joy-Stick x", gamepad1.left_stick_y);
            telemetry.addData("gamepad Joy-Stick y", gamepad1.left_stick_x);
            telemetry.addData("RightENC","(%d,%d)", robot.drivetrain.right_front.getCurrentPosition(), robot.drivetrain.encoderTracker.getRightCurrentPosition());
            telemetry.addData("LeftENC","(%d,%d)", robot.drivetrain.left_front.getCurrentPosition(),robot.drivetrain.encoderTracker.getLeftCurrentPosition());

            telemetry.update();
            prev_gamepad1 = gamepad1;
        }
        //   Robot.mineral_sampler.shutdown();
    }
}


