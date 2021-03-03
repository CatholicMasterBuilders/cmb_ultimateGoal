package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="AccelerometerTest", group= "Linear Opmode")
@Disabled
public class AccelerometerTest extends LinearOpMode {
    Robot robot;
    Drivetrain drivetrain;
    Telemetry telemetry;
    org.firstinspires.ftc.teamcode.Robot.IMU IMU;


    public void runOpMode() {
        robot = new Robot(this);
        drivetrain = new Drivetrain(robot);

        /*------------------------------------------------------------------------------------------
         Initialize all the parameters, drivetain functions, and variable classes when the start button is pressed.
         -------------------------------------------------------------------------------------------*/

        waitForStart();
        robot.drivetrain.drive_init();
        robot.IMU.IMU_calibration();

        /*------------------------------------------------------------------------------------------
         Begins the accelerometer integrating
         -------------------------------------------------------------------------------------------*/



        /*------------------------------------------------------------------------------------------
        Loop to move the robot forward for 1/2 seconds and forward for 1/2 seconds. Repeats 10 times.
        The position of the robot from the accelerometer data is logged after each movement.
        --------------------------------------------------------------------------------------------*/
        while (opModeIsActive()) {

        /*------------------------------------------------------------------------------------------
        Loop to move the robot forward and backwards 10 times,starting at 100 millisecond drives
        and increasing by 100 milliseconds each interval. Waits 2 seconds in between each drive.
        The position from the accelerometer is logged after each movement.
        --------------------------------------------------------------------------------------------*/

            if (gamepad1.y) {
                long loop_interrupt = 0;
                Position position;
                while (opModeIsActive() && loop_interrupt < 1100 /* must be 1100 because it starts at 100.*/) {
                    loop_interrupt += 100;
                    robot.drivetrain.setPower(0.2, 0.2);
                    sleep(loop_interrupt);
                    robot.drivetrain.stopPower();
                    position = robot.IMU.imu.getPosition();
                    RobotLog.dd("Position x, y, z", "%f, %f, %f", position.x, position.y, position.z);
                    sleep(3000);
                    robot.drivetrain.setPower(-0.2, -0.2);
                    sleep(loop_interrupt);
                    robot.drivetrain.stopPower();
                    position = robot.IMU.imu.getPosition();
                    RobotLog.dd("Position x, y, z", "%f, %f, %f", position.x, position.y, position.z);
                    sleep(3000);
                }
            }
        }
    }

}
