package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
import org.firstinspires.ftc.teamcode.DataTypes.Waypoint;
import org.firstinspires.ftc.teamcode.Robot.Robot;

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
public class Waypoint_Nav_BaseEnc extends LinearOpMode {
    Robot robot;
//    SkyStoneField field;

//    int skystoneIndex = 0;
    // Data members
    Coordinate starting_position;

    List<Waypoint> waypoints = new ArrayList<Waypoint>();

    List<Coordinate> skystone_locations = new ArrayList<Coordinate>();

    public enum AutonomousStartLocation {RED_BUILDING, RED_DEPOT, BLUE_BUILDING, BLUE_DEPOT, UNKNOWN; }
    private AutonomousStartLocation autonomousStartLocation;

    public void initialize( Robot robot/*, SkyStoneField field*/ ) {
        this.robot = robot;
//        this.field = field;
    }
    // Special known coordinates
    final int KEEP_CURRENT_COORD = 12345;

    public void runOpMode() {
        robot.init();

        robot.drivetrain.reverseDrivetrain();

        // Try out this method of waiting for start to see if we can leave the robot in an init state
        // for a longer period of time
        //        waitForStart();
        int connection_test = 1;
        int displayed_number = 0;
        telemetry.clearAll();
        while ( !robot.opMode.opModeIsActive() && !robot.opMode.isStopRequested() ) {
            if (connection_test % 500 == 0) {
                displayed_number ++;
            }
            telemetry.addData("Status", "smart waitForStart");
            telemetry.addData("Is changing?", displayed_number);
            telemetry.update();
            connection_test ++;
        }

        // ........And we're ReturnToThree



        // get reference heading to use for navigation ( clockwise is positive )
        double reference_heading = robot.IMU.get_heading();
        robot.heading_adjustment -= reference_heading;
        RobotLog.dd( robot.DBG_TAG, "Reference heading %f\n", reference_heading );
        RobotLog.dd( robot.DBG_TAG, "status %s calib %s", robot.IMU.imu.getSystemStatus().toShortString(), robot.IMU.imu.getCalibrationStatus().toString());

        // Start cruising around waypoints
        for ( int i = 0; i < waypoints.size() && !robot.opMode.isStopRequested(); i++ ) {
            Waypoint current_waypoint = waypoints.get( i );

            // Navigate to next point
            if (current_waypoint.coordinate.x == 12767 && current_waypoint.coordinate.y == 12767){
                RobotLog.dd( robot.DBG_TAG, "Navigate to point %d ( non-manuever )", i);
            }
            else {
                // Check it this is a placeholder for a skystone and insert the actual
                // coordinates here if that is the case
                if ( current_waypoint.coordinate.x == 0 && current_waypoint.coordinate.y == 0 ) {
                    // Skystone1
//                    current_waypoint.coordinate.x = skystone_locations.get(skystoneIndex).x;
//                    current_waypoint.coordinate.y = skystone_locations.get(skystoneIndex).y;
                }

                // Check if either x or y coordinate should be retained
                if ( current_waypoint.coordinate.x == KEEP_CURRENT_COORD ) {
                    current_waypoint.coordinate.x = robot.robot_coord.x;
                }
                if ( current_waypoint.coordinate.y == KEEP_CURRENT_COORD ) {
                    current_waypoint.coordinate.y = robot.robot_coord.y;
            }

                RobotLog.dd( robot.DBG_TAG, "Navigate to point %d ( %d, %d )", i, current_waypoint.coordinate.x, current_waypoint.coordinate.y);
                robot.maneuver.maneuverToPoint(robot.robot_coord, current_waypoint.coordinate);
            }

            // Execute additional actions if there are any
            if ( current_waypoint.action != null ) {
                current_waypoint.action.run();
            }
        }

        // Clean up
    }

}