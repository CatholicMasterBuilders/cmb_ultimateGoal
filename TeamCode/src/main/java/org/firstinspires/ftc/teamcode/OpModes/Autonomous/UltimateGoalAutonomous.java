package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
import org.firstinspires.ftc.teamcode.DataTypes.Waypoint;
import org.firstinspires.ftc.teamcode.Robot.Robot;

@Autonomous(name = "UltimateGoal Autonomous", group = "Linear Opmode")
public class UltimateGoalAutonomous extends Waypoint_Nav_Base {
    Robot robot;

    Gamepad prev_gamepad1 = new Gamepad();

    int alliance_mirror_adjustor = 1; // Specifies how to adjust field coords and rotations for red and blue


    public AutonomousStartLocation autoStartingLocation = AutonomousStartLocation.RED_LEFT;

    @Override
    public void runOpMode (){
        robot = new Robot(this);
        telemetry.addLine("Please enter Auto start Location \n" +
                            "gamepad x => RED LEFT \n" +
                            "gamepad b => RED RIGHT");
        telemetry.update();

        while (1 == 1){

            if (prev_gamepad1.x && gamepad1.x){
                telemetry.addLine("Auto start Position =>" + autoStartingLocation.toString());
                break;
            }

            if (prev_gamepad1.b && gamepad1.b){
                autoStartingLocation = AutonomousStartLocation.RED_RIGHT;
                telemetry.addLine("Auto start Position =>" + autoStartingLocation.toString());
                break;
            }

            prev_gamepad1 = gamepad1;
        }
        RobotLog.dd("CMBRR", "robot x pos %d", robot.robot_coord.x);
        RobotLog.dd("CMBRR", "robot y pos %d", robot.robot_coord.y);
        telemetry.update();

        initializeKnownLocations(robot, autoStartingLocation);

        //Parking waypoint
        waypoints.add(new Waypoint(new Coordinate(920 * alliance_mirror_adjustor,915)));

//        //Picking up rings waypoint TODO: Needs Runnable to move motors to intake the rings
//        waypoints.add(new Waypoint(new Coordinate(920 * alliance_mirror_adjustor, -420)));
//
//        //Wobble Goal delivery waypoint
//        waypoints.add(new Waypoint(new Coordinate(885 * alliance_mirror_adjustor, 910)));

        super.runOpMode();
    }

}
