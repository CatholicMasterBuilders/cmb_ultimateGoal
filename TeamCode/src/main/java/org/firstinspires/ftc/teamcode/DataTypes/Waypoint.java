package org.firstinspires.ftc.teamcode.DataTypes;

public class Waypoint {

    public Coordinate coordinate;
    public Heading heading;
    public Runnable action;
    public Waypoint ( Coordinate coord ) {
        this( coord, new Heading(), null );
    }
    public Waypoint ( Coordinate coord, Heading heading ) {
        this( coord, heading, null );
    }
    public Waypoint ( Coordinate coord, Runnable act) {
        this( coord, new Heading(), act );
    }
    public Waypoint ( Coordinate coord, Heading heading, Runnable act ) {
        coordinate = coord;
        this.heading = heading;
        action = act;
    }
}