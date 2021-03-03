package org.firstinspires.ftc.teamcode.Navigation;

import org.firstinspires.ftc.teamcode.DataTypes.Arc;
import org.firstinspires.ftc.teamcode.DataTypes.Coordinate;
import org.firstinspires.ftc.teamcode.Robot.Robot;

public class ArcPath {

    public ArcPath (Robot robot) {setRobot(robot);}

    //Variables
    Robot robot = getRobot();
    Arc arc = new Arc(0,0, 0);
    double robot_slope;
    double robot_radius_slope;
    double target_slope;
    double target_radius_slope;
    double b_robot;
    double b_target;
    double b_robot_radius;
    double b_target_radius;
    Coordinate circle_center;
    Coordinate fred;
    double jiraffery;
    double akbar;

    static final double PI = 3.14159265359;



    /**
     * This API uses the robot coordinate and heading and the target position and heading to find
     * the arc of the circle that the robot must drive to get to a certain point and heading. It
     * will always be the arc of a circle.
     * @param target_position the desired coordinate and heading of the robot.
     * @return the radius and the angle of the arc the robot must drive along
     */
    public Arc getArc(Coordinate target_position) {
        //Finds the slops of all the lines
        robot_slope = Math.tan(robot.emu.get_x_axis_heading());
        robot_radius_slope = -1 / robot_slope;
        target_slope = Math.tan(target_position.heading);
        target_radius_slope = -1 / target_slope;

        //Finds the y intercept of all the lines
        b_robot = robot.robot_coord.y - (robot_slope * robot.robot_coord.x);
        b_robot_radius = robot.robot_coord.y - (robot_radius_slope * robot.robot_coord.x);
        b_target = target_position.y - (target_slope * target_position.x);
        b_target_radius = target_position.y - (target_radius_slope * target_position.x);

        /*Finds the x and y coordinates of the center of the circle by solving the system of equations
         for both the radii by using the substitution method.*/
        circle_center.x = (int) ((b_robot_radius - b_target_radius) / (target_radius_slope - robot_radius_slope));
        circle_center.y = (int) (target_radius_slope * circle_center.x + b_target_radius);

        //!!BIG TIME!! Finds the radius of the circle by using the Pythagorean Theorem
        arc.radius = Math.hypot(circle_center.x - target_position.x, circle_center.y - target_position.y);

        arc.length = 2 * PI * arc.radius;

        /*Finds the x and y coordinates of the point where both the tangent lines of the arc intercept
        by solving the system of equations of the robot heading line and target posiiton heading line.*/
        fred.x = (int) ((b_target - b_robot) / (robot_slope - target_slope));
        fred.y = (int) (robot_slope * fred.x + b_robot);


        /*Finds the length of the tangent line between the target point and the point where the two
        tangent lines intersect by using the Pythagorean Theorem.*/
        akbar = Math.hypot(fred.x - target_position.x, fred.y - target_position.y);

        jiraffery = Math.toDegrees(Math.atan(akbar / arc.radius));

        arc.angle = jiraffery * 2;

        return arc;
    }


    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }
}
