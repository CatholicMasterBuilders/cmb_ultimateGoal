package org.firstinspires.ftc.teamcode.DataTypes;

public class Heading {
    public enum Type { DEGREES, RADIANS };
    private double heading;
    private boolean is_heading_valid;
    public Heading() {
        setHeadingInvalid();
    }
    public Heading( double heading ) {
        setHeadingValid();
        this.heading = toNormalizedHeading( heading );
    }

    /**
     * Convert heading to a normalized value in the range (-179,180]
     * @param heading input heading
     * @return heading in the normalized range
     */
    public static double toNormalizedHeading( double heading ) {
//        return heading - Math.ceil( heading/360 - 0.5 )*360;
        return internalNormalizeHeading( Type.DEGREES, heading );
    }
    public static double toNormalizedHeading( Type type, double heading ) {
//        return heading - Math.ceil( heading/360 - 0.5 )*360;
        return internalNormalizeHeading( type, heading );
    }
    private static double internalNormalizeHeading( Type type, double heading ) {
        double rangeSize = ( type == Type.DEGREES ) ? 360 : 2*Math.PI;
        return heading - Math.ceil( heading/rangeSize - 0.5 )*rangeSize;
    }

    /**
     * Convert to compass value in the range [0,360]
     * @param heading input heading
     * @return heading in the compass range
     */
    public static double toCompassHeading( double heading ) {
        return ( heading + 360 ) % 360;
    }

    public boolean isHeadingValid() { return is_heading_valid; }

    public double toDegrees() { return heading; }
    public double toRadians() { return Math.toRadians( heading ); }

    private void setHeadingInvalid( ) { is_heading_valid = false; }
    private void setHeadingValid( ) { is_heading_valid = true; }
}