package org.firstinspires.ftc.teamcode.Navigation;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.DataTypes.Arc;
import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
import org.firstinspires.ftc.teamcode.Robot.Robot;

import static org.firstinspires.ftc.teamcode.DataTypes.Heading.toNormalizedHeading;

public class Maneuver {
    Robot robot;

    public Maneuver(Robot robot) {
        setRobot(robot);
    }

    double turn_power = 0.25;
    private double drive_power = 0.4;
    double mini_arc_size = 0; //I am not sure if I want to use this.
    double last_partial_mini_arc = 0;
    double carl = 0; //The side when added to the change in x equals the radius...I had no clue what to call it.
    double change_in_y = 0; //The y change between the two endpoints of the arc.
    double change_in_x = 0; //The x change between the two endpoints of the mini arc

    private double speed_ratio = 0;
    private double right_motor_speed = 0;
    private Arc arc = new Arc(0,0,0);

    static final int NUMBER_OF_MINI_ARCS = 10;
    static final double ENC_WHEEL_TRACK = 10; //Total uneducated guess
    static final double PI = 3.14159265359;



    public boolean maneuverByArcToPoint (Coordinate target_position, double base_speed){

        arc = robot.arcPath.getArc(target_position);

        mini_arc_size = arc.angle / NUMBER_OF_MINI_ARCS;

        //last_partial_mini_arc = arc.angle % MINI_ARC_SIZE -- I really wanted to use mod division

        carl = Math.cos(Math.toRadians(mini_arc_size)) * arc.radius;

        change_in_x = arc.radius - carl; //The thought popped in my head to absolute value this...but I don't think I should.

        change_in_y = Math.sin(Math.toRadians(mini_arc_size)) * arc.radius;

        speed_ratio = change_in_y / change_in_x;

        right_motor_speed = speed_ratio * base_speed;


        robot.drivetrain.setPower(base_speed, right_motor_speed);

        while (robot.drivetrain.getDistanceTravelled() < arc.length){
            ;
        }

        robot.drivetrain.setPower(0,0);

        return true;
    }




    public boolean maneuverToPoint (Coordinate robot_position, Coordinate target_position){
        boolean arrived = false;
        RobotLog.dd("CMBRRR", "manueverToPoint: robot_position (x,y)=(%d,%d) target_position (x,y) = (%d,%d)",
                robot.robot_coord.x, robot.robot_coord.y, target_position.x, target_position.y);
        if (turnToCoordinate(robot_position, target_position) && driveToCoordinate(robot_position, target_position)){
            arrived = true;
        }
        RobotLog.dd("CMBRRR", "manueverToPoint: target_position (x,y)=(%d,%d) actual_position (x,y) = (%d,%d)",
                target_position.x, target_position.y, robot.robot_coord.x, robot.robot_coord.y);
        return arrived;
    }

    //This will always make the front face the desired point.
    private boolean turnToCoordinate (Coordinate robot_position, Coordinate target_position){
        double desired_heading = robot.path.getHeading(robot_position, target_position);
        double current_heading = robot.IMU.get_x_axis_heading();
        double turn_distance = toNormalizedHeading( desired_heading - current_heading );

        RobotLog.dd("CMBRRR", "turnToCoordinate: heading (current,desired) = (%.4f,%.4f) turnDistance = %f", current_heading, desired_heading, turn_distance);


        // TODO: Try going back to using the IMU for the turn since the encoder based turns won't work anymore
        // Note the turn adjustment has been pushed into the turn method

      robot.IMU.turn(0.25, turn_distance);
//      robot.IMU.turn_to_heading(turn_power, desired_heading);


        RobotLog.dd("CMBRR", "turnToCoordinate: turned");
        return true;
    }

    private boolean driveToCoordinate (Coordinate robot_position, Coordinate target_position){
        double distance = robot.path.getDistance(robot_position, target_position);
//        robot.drivetrain.encoder_drive_millimeters(distance, distance, robot_position, target_position, drive_power, drive_power);
        robot.drivetrain.encoder_drive_mm(distance, distance, drive_power, drive_power);

        RobotLog.dd("CMBRR", "driveToCoordinate: drove");
        return true;
    }

    // Getters and Setters
    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

    public double getDrive_power() { return drive_power; }

    public void setDrive_power(double drive_power) { this.drive_power = drive_power; }

}
