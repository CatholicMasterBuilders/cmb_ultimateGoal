package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot.Robot;

import static org.firstinspires.ftc.teamcode.DataTypes.Heading.toCompassHeading;
import static org.firstinspires.ftc.teamcode.DataTypes.Heading.toNormalizedHeading;

@Autonomous (name = "IMULagTest", group = "LinearOpMode")
@Disabled
public class IMULagTest extends LinearOpMode {

    Robot robot;

    public void runOpMode(){
        robot = new Robot(this);
//        robot.init();
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        int iterCnt;
        double s,e,d;
        time.reset();
        for ( iterCnt = 0; iterCnt < 1000; ++iterCnt ) {
            // This loop doesn't do much but some math
            s = toCompassHeading( iterCnt ) + ( iterCnt % 13 );
            s = toNormalizedHeading( s );
//            e = robot.IMU.toNormalizedHeading( iterCnt );
            e = robot.IMU.get_raw_heading();
            d = toNormalizedHeading(e - s);
            RobotLog.dd("CMBRR", "junk log %3.0f", d);
        }
        double endTime = time.milliseconds();
        RobotLog.dd("CMBRR", "Executed %d loops in %f milliseconds = %3.0f ms/loop", iterCnt, endTime, endTime/iterCnt);

//        double turn_millimeters = Math.abs( ( (17*25.4) * 2 * Math.PI ) * (90 / 360) ) / 2; // This should be about a 90 degree turn
//
//        double start_heading = robot.IMU.get_heading();
//        RobotLog.dd("CMBRR", "IMULagTest: Start Heading %f", start_heading );
//        robot.drivetrain.encoder_drive_millimeters(-turn_millimeters, turn_millimeters,1.0,1.0,0, 45);
//        time.reset();
//        while ( time.milliseconds() < 1000 ){
//            RobotLog.dd("CMBRR", "IMULagTest: settling down heading %f degrees", robot.IMU.get_heading());
//            sleep( 100 );
//        }
//        RobotLog.dd("CMBRR", "IMULagTest: final heading %f degrees turned %f", robot.IMU.get_heading(), start_heading - robot.IMU.get_heading());
//
//        start_heading = robot.IMU.get_heading();
//        RobotLog.dd("CMBRR", "IMULagTest: Start Heading %f", start_heading );
//        robot.drivetrain.encoder_drive_millimeters(turn_millimeters, -turn_millimeters,1.0,1.0,0, 45);
//        time.reset();
//        while ( time.milliseconds() < 1000 ){
//            RobotLog.dd("CMBRR", "IMULagTest: settling down heading %f degrees", robot.IMU.get_heading());
//            sleep( 100 );
//        }
//        RobotLog.dd("CMBRR", "IMULagTest: final heading %f degrees turned %f", robot.IMU.get_heading(), start_heading - robot.IMU.get_heading());
    }
}
