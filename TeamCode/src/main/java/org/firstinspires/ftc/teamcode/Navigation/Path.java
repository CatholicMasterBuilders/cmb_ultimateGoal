package org.firstinspires.ftc.teamcode.Navigation;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class Path {

    public Path(Robot robot) {
        setRobot(robot);
    }

    Robot robot = getRobot();

    public final double CENTER_OF_TURN_FROM_FRONT_MILLIMETERS = 10.5 * 25.4; //If you've got a better name, please change it.

//    public int getPathSlope (Coordinate robot_position, Coordinate target_position){
//        int change_in_y = target_position.y - robot_position.y;
//        int change_in_x = target_position.x - robot_position.x;
//        int path_slope = change_in_y / change_in_x;
//
//        return path_slope;
//    }

    public double getHeading(Coordinate robot_position, Coordinate target_position) {
//adb disconnect double slope = getPathSlope(target_position, robot_position);
//        boolean turn_needed = true;
        int change_in_y = target_position.y - robot_position.y;
        int change_in_x = target_position.x - robot_position.x;
        double distance = Math.hypot(change_in_x, change_in_y);
        double radian_heading = Math.atan2(change_in_y, change_in_x);
        double heading = Math.toDegrees(radian_heading);
        double actual_heading = robot.IMU.get_x_axis_heading();
        if (target_position.direction == Coordinate.Direction.BACKWARD) {
            if (distance < CENTER_OF_TURN_FROM_FRONT_MILLIMETERS) {
                //Check to see which direction the robot is going
                int front_position_x = (int) (Math.cos(Math.toRadians(actual_heading)) * CENTER_OF_TURN_FROM_FRONT_MILLIMETERS + robot.robot_coord.x);
                int front_position_y = (int) (Math.sin(Math.toRadians(actual_heading)) * CENTER_OF_TURN_FROM_FRONT_MILLIMETERS + robot.robot_coord.y);
                if (Math.hypot(front_position_x - target_position.x, front_position_y - target_position.y) < CENTER_OF_TURN_FROM_FRONT_MILLIMETERS) {
                //Target position is in front of the middle point but behind the front
                }
                else {
                    heading -= 180;
                }
            }
            else {
                //Adjust heading for backwards
                heading -= 180;
            }
        }

//        double opp_ang = 90 - base_heading;
//
//        if (slope == 0 || robot_position.x == target_position.x){
//            turn_needed = false;
//        }
//        else if(turn_needed && slope < 0 && robot_position.y < target_position.y){
//            heading = (opp_ang * -1) - 90;
//        }
//        else if (turn_needed && slope > 0 && robot_position.y < target_position.y){
//            heading = base_heading * -1;
//        }
//        else if (turn_needed && slope > 0 && robot_position.y > target_position.y){
//            heading = opp_ang + 90;
//        }
//        else if (turn_needed && slope < 0 && robot_position.y > target_position.y){
//            heading = base_heading;
//        }
        RobotLog.dd("CMBRR", "robot.path.getHeading: desired_heading %f", heading);

        return heading;
    }

    public double getDistance(Coordinate robot_position, Coordinate target_position){
            double leg1 = target_position.y - robot_position.y;
            double leg2 = target_position.x - robot_position.x;
            double distance = Math.hypot(leg1, leg2);
        RobotLog.dd("CMBRR", "Path.getDistance distance %f direction:"+target_position.direction, distance);
            if (distance < CENTER_OF_TURN_FROM_FRONT_MILLIMETERS && target_position.direction == Coordinate.Direction.BACKWARD) {

                double heading = robot.IMU.get_x_axis_heading();

                int front_position_x = (int) (Math.cos(Math.toRadians(heading)) * CENTER_OF_TURN_FROM_FRONT_MILLIMETERS + robot.robot_coord.x);
                int front_position_y = (int) (Math.sin(Math.toRadians(heading)) * CENTER_OF_TURN_FROM_FRONT_MILLIMETERS + robot.robot_coord.y);

                RobotLog.dd("CMBRR", "Path.getDistance: in magic threshold");
                // The first if block I am pretty sure already ensures the point is not past the front of the robot. So, we only need to check
                //if it's past the middle point.
                double hypot = Math.hypot(front_position_x - target_position.x, front_position_y - target_position.y);
                RobotLog.dd("CMBRR", "Path.getDistance hypot between front and center %f", hypot);
                if (hypot < CENTER_OF_TURN_FROM_FRONT_MILLIMETERS) {
                    distance -= CENTER_OF_TURN_FROM_FRONT_MILLIMETERS;
                    RobotLog.dd("CMBRR", "Path.getDistance: between center and front");

                } else {
                    if (target_position.direction == Coordinate.Direction.BACKWARD) {
                        distance *= -1;
//                        distance -= CENTER_OF_TURN_FROM_FRONT_MILLIMETERS;
                        RobotLog.dd("CMBRR", "Path.getDistance: just going backwards");
                    }
                }
            }
            else if (target_position.direction == Coordinate.Direction.BACKWARD) {
                distance *= -1;
//                distance -= CENTER_OF_TURN_FROM_FRONT_MILLIMETERS;
                RobotLog.dd("CMBRR", "Path.getDistance: just going backwards big distance");
            }
            else {
//                distance -= CENTER_OF_TURN_FROM_FRONT_MILLIMETERS;
                RobotLog.dd("CMBRR", "Path.getDistance: Going forwards");
            }
        RobotLog.dd("CMBRR", "robot.path.getDistance: desired_distance %f(mm) (%f in)", distance, distance/25.4);
        return distance;
    }

    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }
}
