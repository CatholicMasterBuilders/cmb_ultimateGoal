package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
import org.firstinspires.ftc.teamcode.DataTypes.Waypoint;
import org.firstinspires.ftc.teamcode.Robot.Robot;
//import org.firstinspires.ftc.teamcode.Vision.SkystoneDetectorInterface;

import java.util.ArrayList;
import java.util.List;

// This class is a base class for all of our autonomous programs to extend
// It implements using our point to point navigation system to run the
// autonomous program.  Classes which extend this class just need to choose the initialization
// location and then set up the list of waypoints for the robot to complete its autonomous
// operation.
// The location for the gold mineral will be inserted into the list of waypoints by this base
// class, so individual autonomous classes don't need to know anything about sampling
// If an autonomous class wants to do extra actions at any given waypoint ( such as deploying
// the team marker ), then they can provide a Runnable which will be run when the waypoint is
// reached.
@Autonomous (name = "Waypoint_Nav_Base", group = "LinearOpMode")
@Disabled /* This class can't actually be run */
public class Waypoint_Nav_Base extends LinearOpMode {
    Robot robot;

    int skystoneIndex = 0;
    // Data members
    Coordinate starting_position;

    List<Waypoint> waypoints = new ArrayList<Waypoint>();

    List<Coordinate> skystone_locations = new ArrayList<Coordinate>();

    // Coordinates with special meaning
    Coordinate NonNavCoordinate = new Coordinate(12767, 12767 );

    public enum AutonomousStartLocation {RED_LEFT, RED_RIGHT, BLUE_LEFT, BLUE_RIGHT}
    private AutonomousStartLocation autonomousStartLocation;
    public void initializeKnownLocations(Robot robot, AutonomousStartLocation staringSide ) {
        this.robot = robot;
        autonomousStartLocation = staringSide;

        switch (staringSide) {
            case RED_RIGHT:
                robot.set_robot_coord(1253, -1828 + robot.cog_offset_from_back);
                robot.heading_adjustment = 0;

                break;
            case BLUE_LEFT:
               //Not necessary right now.

                break;
            case BLUE_RIGHT:
                //Not necessary right now.

                break;

            case RED_LEFT:
            default:
                robot.set_robot_coord(650, -1828 + robot.cog_offset_from_back);
                robot.heading_adjustment = 0;

                break;
        }
        RobotLog.dd(robot.DBG_TAG, "robot position after initialization (x,y) = (%d,%d)", robot.robot_coord.x, robot.robot_coord.y);

    }

    // Special known coordinates
    final int KEEP_CURRENT_COORD = 12345;

    public void runOpMode() {
        robot.init();

        // get reference heading to use for navigation ( clockwise is positive )
        double reference_heading = robot.IMU.get_heading();
        robot.heading_adjustment -= reference_heading;

        RobotLog.dd( robot.DBG_TAG, "Reference heading %f\n", reference_heading );
        RobotLog.dd( robot.DBG_TAG, "status %s calib %s", robot.IMU.imu.getSystemStatus().toShortString(), robot.IMU.imu.getCalibrationStatus().toString());

        waitForStart();

        // Start cruising around waypoints
        for ( int i = 0; i < waypoints.size() && !robot.opMode.isStopRequested(); i++ ) {
            Waypoint current_waypoint = waypoints.get( i );

            // Navigate to next point

                RobotLog.dd( "CMBRRR", "Navigate to point %d ( %d, %d )", i, current_waypoint.coordinate.x, current_waypoint.coordinate.y);
                robot.maneuver.maneuverToPoint(robot.robot_coord, current_waypoint.coordinate);


            // Execute additional actions if there are any
            if ( current_waypoint.action != null ) {
                current_waypoint.action.run();
            }
        }

        // Clean up
        //robot.skystone_detector.shutdown();
    }

}