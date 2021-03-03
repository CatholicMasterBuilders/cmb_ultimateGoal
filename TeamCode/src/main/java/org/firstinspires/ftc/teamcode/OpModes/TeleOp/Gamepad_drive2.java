package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Robot;

//import org.opencv.core.Range;

@TeleOp (name = "Gamepad_drive2")
public class Gamepad_drive2 extends LinearOpMode {
    Robot robot;
    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;
    boolean isPowerShots = false;
    boolean isChongusOff = true;
    boolean xPressed = false;

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
        robot.placingSystem.thingy.setPower(-0.5);
        sleep(800);
        robot.placingSystem.thingy.setPower(0);
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

            if (gamepad2.x){
                xPressed = true;
            }

            //Setter
            if (gamepad1.b){
                robot.launcher.five_ThreeSir_Three();
            }


            else {
                robot.launcher.hitFourYouHaveGoneTooFar();
            }


            if (gamepad1.right_trigger > 0.1) {
                robot.intake.slurp();
                robot.launcher.pullHolyGranadePin();
            }

            else if (gamepad1.left_trigger > 0.1){
                robot.intake.spit();
            }

            else {
                robot.intake.salivate();
                if (isChongusOff)
                    robot.launcher.holyHandGranadeManuel();
            }



            //Controller 2



            //Dingus Kahn (Wobble goal grabber) controls
            if (gamepad2.right_trigger > 0.1) {
                robot.placingSystem.openDingusKahn();
            }

            else
                robot.placingSystem.closeDingusKahn();

            //Placing arm controls
            if (gamepad2.right_stick_y == 0){
                robot.placingSystem.thingy.setPower(-0.2);
            }

            else
                robot.placingSystem.setArmPower(gamepad2.right_stick_y);


            if (gamepad2.right_bumper){
                rightBumperPressed = true;
            }

            //Turning chongus on or off
            if (!gamepad2.right_bumper && rightBumperPressed){
                //If chongus is off, turn it on
                if(isChongusOff) {
                    robot.launcher.one();
                    isChongusOff = false;
                }

                //If chongus is on, turn it off.
                else if (!isChongusOff){
                    robot.launcher.holyHandGranadeManuel();
                    isChongusOff = true;
                }
                rightBumperPressed = false;
            }

            if (gamepad2.left_bumper){
                leftBumperPressed = true;
            }

            //Turning chongus on or off
            if (!gamepad2.left_bumper && leftBumperPressed){
                //If chongus is off, turn it on
                if(isChongusOff) {
                    robot.launcher.one(-0.7);
                    isChongusOff = false;
                }

                //If chongus is on, turn it off.
                else if (!isChongusOff){
                    robot.launcher.holyHandGranadeManuel();
                    isChongusOff = true;
                }
                leftBumperPressed = false;
            }



            //Aimer has three set positions
            if (gamepad2.dpad_up){
                robot.launcher.two(0.435);
            }

            else if (gamepad2.dpad_down){
                robot.launcher.two(0);
            }

            else if (gamepad2.dpad_left || gamepad2.dpad_right)
                robot.launcher.two(0.415);

            if (gamepad2.b) {
                robot.launcher.setScorpion_tail(0);

            }

            else {
                robot.launcher.setScorpion_tail();
            }


            if (gamepad1.start) {
                isPowerShots = true;
            }

            if (isPowerShots) {

                robot.drivetrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.drivetrain.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.launcher.two(0.43);
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
        }
        //   Robot.mineral_sampler.shutdown();
    }
}