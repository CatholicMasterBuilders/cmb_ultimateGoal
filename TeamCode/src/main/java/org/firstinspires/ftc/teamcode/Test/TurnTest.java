package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="TurnTest", group= "Linear Opmode")
@Disabled
public class TurnTest extends LinearOpMode {
    Robot robot;
    private int waitForGyroToSettle() {
        double prev_heading = 0;
        double current_heading = robot.IMU.get_heading();
        double stability_threshold = 1;
        int stability_loop_cnt = 0;
        while ( Math.abs( current_heading - prev_heading ) > stability_threshold ) {
            sleep( 100 );
            prev_heading = current_heading;
            current_heading = robot.IMU.get_heading();
            stability_loop_cnt++;
        }
        return stability_loop_cnt*100;
    }

    public void runOpMode() {
        robot = new Robot(this);

        robot.init();

        telemetry.addLine( "Hit the start button and stand back");
        waitForStart();

        double turn_speed;
        double degrees_to_turn;
        double start_heading;
        int stabilization_time;
        for (int speed_step = 3; opModeIsActive() && speed_step <= 10; speed_step++) {
            turn_speed = 0.05 * speed_step;
            for (int degree_step = 1; opModeIsActive() && degree_step <= 17; degree_step++) {
                if (degree_step <= 8) degrees_to_turn = 5 * degree_step; // 0 to 40 in 5 degree chunks
                else if (degree_step <= 14) degrees_to_turn = 40 + 10 * (degree_step - 8);// 40 to 160 in 10 degree chunks
                else degrees_to_turn = 100 + 20 * (degree_step-14);

                telemetry.addLine()
                        .addData(" Test: power ", turn_speed)
                        .addData(" degrees ", degrees_to_turn );
                telemetry.update();

                robot.IMU.turn(turn_speed, -degrees_to_turn);
                start_heading = robot.IMU.get_compass_heading();
                stabilization_time = waitForGyroToSettle();
                RobotLog.dd(robot.DBG_TAG, "after turn(%f,%f) gyro stabalized %f degrees in %d milliseconds\n", turn_speed, degrees_to_turn, robot.IMU.getHeadingDelta(start_heading), stabilization_time);

                // Forced pause between each test so we don't blow anything up
                sleep(1000 - stabilization_time);
            }
        }
    }

}
