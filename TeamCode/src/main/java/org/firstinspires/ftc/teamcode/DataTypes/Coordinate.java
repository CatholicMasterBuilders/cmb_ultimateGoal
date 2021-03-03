package org.firstinspires.ftc.teamcode.DataTypes;

public class Coordinate {
//     If you are standing in the Red Alliance Station looking towards the center of the field,
//    - The X axis runs from your left to the right. (positive from the center to the right)
//    - The Y axis runs from the Red Alliance Station towards the other side of the field
//     where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)

    public int x;
    public int y;
    public Direction direction;
    public double heading;
    public double exact_x;
    public double exact_y;

    private static final int cor_offset_from_front = (int)(10.5 * 25.4);
    private static final int cor_offset_from_back = (int)(18 * 25.4) - cor_offset_from_front;

    public enum Direction {FORWARD, BACKWARD;}

    public Coordinate(int x, int y, Direction direction) {
        this.x = x;
        this.y = y;
        this.direction = direction;
    }

    public Coordinate(int x, int y) {
        this.x = x;
        this.y = y;
        this.direction = Direction.FORWARD;
    }

    public Coordinate (int x, int y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }


    public void updateLocation( int x, int y ) {
        this.x = x;
        this.y = y;
    }
    public void updateLocation( Coordinate coord ) {
        x = coord.x;
        y = coord.y;
    }
    public void updateCoordinate( Coordinate coord ) {
        updateLocation( coord );
        direction = coord.direction;
    }

    @Override
    public boolean equals( Object o ) {
        Coordinate c = (Coordinate) o;
        return ( o == this ) || (x == c.x && y == c.y );
    }

    public static Coordinate getAdjustedCoord( int x, int y, Heading h, boolean b) {
        Coordinate coord = new Coordinate(x,y);

        int offset = ( b == true ) ? cor_offset_from_front : cor_offset_from_back;
        x += Math.cos( h.toRadians() ) * offset;
        y += Math.sin( h.toRadians() ) * offset;

        coord.updateLocation( x, y );
        return coord;
    }
}