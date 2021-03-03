package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Launcher;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp (name = "ServoLimitFinder")
//@Disabled
public class ServoLimitFinder extends LinearOpMode {
    Robot robot;
    // servo class
    CRServo test_crservo;
    Servo test_servo;
    Servo test_servo_2;

    public void runOpMode() {
        robot = new Robot(this);

        double servo_test_position;
        double servo_test_position_2;



        // **************************************************************************************
        // Get the Servo by updating the type of servo and the name
        // **************************************************************************************
        final int servo_type = 1; // 0 for CRServo, 1 for regular servo
        final boolean second_servo = false;
        if ( servo_type == 0 ){
            test_crservo = hardwareMap.get(CRServo.class,"aimer");
        }
        else {
            test_servo = hardwareMap.get(Servo.class,"grabber");
            if (second_servo){
                test_servo_2 = hardwareMap.get(Servo.class, "right_lock");
                test_servo_2.setDirection(Servo.Direction.REVERSE);
            }

         //   test_servo.setDirection(Servo.Direction.REVERSE);
        }

        servo_test_position = test_servo.getPosition();
//        servo_test_position_2 = test_servo_2.getPosition();

        telemetry.log().setCapacity(4);
        telemetry.log().add("Press x to increase the servo position");
        telemetry.log().add("Press y to decrease the servo position");
//        telemetry.log().add("Press a to increase the servo 2 position");
//        telemetry.log().add("Press b to decrease the servo 2 position");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Adjust the test positions
            if (gamepad1.x ){servo_test_position = servo_test_position + 0.01;}
            if (gamepad1.y){servo_test_position = servo_test_position - 0.01;}
//            if (gamepad1.a) {servo_test_position_2 += 0.01;}
//            if (gamepad1.b) {servo_test_position_2 -= 0.01;}

            // set the test position based on servo type
            if ( servo_type == 0 ) {
                test_crservo.setPower(com.qualcomm.robotcore.util.Range.clip( servo_test_position, -1, 1));
            }
            else {
                test_servo.setPosition( servo_test_position );

                if (second_servo){
                    test_servo_2.setPosition( servo_test_position_2 );
                }
            }

            // add some telmetry based on servo type
            telemetry.addData("servo_test_position %f", servo_test_position);
            if ( servo_type == 0 ) {
                telemetry.addData("servo position %f", test_crservo.getPower());
            }
            else {
                telemetry.addData("servo position", test_servo.getPosition());

                if (second_servo){
                    telemetry.addData("servo 2 position", test_servo_2.getPosition());
                }
            }

            if (gamepad1.a){
                robot.launcher.one();
            }

            if (gamepad1.b){
                robot.launcher.five_ThreeSir_Three();
            }

            else {
                robot.launcher.hitFourYouHaveGoneTooFar();
            }
            telemetry.update();
            sleep(50);
        }
    }

}
