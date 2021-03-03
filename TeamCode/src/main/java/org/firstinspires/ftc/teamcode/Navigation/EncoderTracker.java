package org.firstinspires.ftc.teamcode.Navigation;


import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderTracker {
    // For all of these APIs, INCREMENTAL means that they are only reporting the incremental
    // changes based on the last time update() was called.
    // TOTAL means they are reporting the total change since the last time init() was called
    public enum DeltaType { INCREMENTAL, TOTAL };

    private int left_current;
    private int left_previous;
    private int left_start;
    private int right_current;
    private int right_previous;
    private int right_start;
    private DcMotor left_encoder;
    private DcMotor right_encoder;
    private int leftDir = 1;
    private int rightDir = -1;
    private boolean left_first = true;

    public EncoderTracker( DcMotor left, DcMotor right ) {
        // Setup the encoders to be used for this tracker
        left_encoder = left;
        right_encoder = right;
    }

    public double getAverage( DeltaType type ) {
        return   (double)(getLeftEncoderDelta( type ) + getRightEncoderDelta( type )) / 2.0;
    }
    public int getDifference( DeltaType type ) {
        return  (int) (getLeftEncoderDelta( type ) - getRightEncoderDelta( type ));
    }

//    double TRACKING_WHEEL_TICK_DIFFERENTIAL_PER_ONE_DEGREE_OF_HEADING_DELTA = 24.8;
    double TRACKING_WHEEL_TICK_DIFFERENTIAL_PER_ONE_DEGREE_OF_HEADING_DELTA = 24.313725;
    public double getEncoderHeadingDelta( DeltaType type ) {
        return -getDifference( type ) / TRACKING_WHEEL_TICK_DIFFERENTIAL_PER_ONE_DEGREE_OF_HEADING_DELTA;
    }

    public int getLeftEncoderDelta( DeltaType type ) {
        switch ( type ) {
            case TOTAL:
                return left_current - left_start;
            case INCREMENTAL:
            default:
                return left_current - left_previous;
        }
    }
    public int getRightEncoderDelta( DeltaType type ) {
        switch ( type ) {
            case TOTAL:
                return right_current - right_start;
            case INCREMENTAL:
            default:
                return right_current - right_previous;
        }
    }

    int right,left;
    public void update() {
        if ( left_first ) {
            left = getLeftCurrentPosition();
            right = getRightCurrentPosition();
        } else {
            right = getRightCurrentPosition();
            left = getLeftCurrentPosition();
        }
        update( left, right );
        left_first = !left_first;
    }
    private void update(int left_position, int right_position) {
        left_previous = left_current;
        left_current = left_position;
        right_previous = right_current;
        right_current = right_position;
    }
    public void init() {
        left_current = getLeftCurrentPosition();
        left_previous = left_current;
        left_start = left_current;
        right_current = getRightCurrentPosition();
        right_previous = right_current;
        right_start = right_current;
    }
    public int getRightCurrentPosition() {
        return right_encoder.getCurrentPosition() * rightDir;
    }
    public int getLeftCurrentPosition() {
        return left_encoder.getCurrentPosition() * leftDir;
    }
}