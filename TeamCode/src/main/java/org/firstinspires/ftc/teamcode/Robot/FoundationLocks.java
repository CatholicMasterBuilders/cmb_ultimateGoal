package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FoundationLocks {

    //Data handles to ftc app
    Robot robot;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    LinearOpMode opmode;

    //Locks
    Servo right_lock;
    Servo left_lock;

    // Internal variables
    boolean lock_state = false;
    private final double LOCK_POSITION_RIGHT = 0.42;
    private final double RELEASE_POSITION_RIGHT = 1.0;
    private final double LOCK_POSITION_LEFT = 0.28;
    private final double RELEASE_POSITION_LEFT = 0.86;

    // Constructor(s)
    public FoundationLocks(Robot robot) {
        setRobot( robot );
    }

    // All the usual initialization stuff
    public void init (){
        hardwareMap = getRobot().hardwareMap;
        telemetry = getRobot().telemetry;
        opmode = getRobot().opMode;

        right_lock = hardwareMap.get(Servo.class, "right_lock");
        left_lock = hardwareMap.get(Servo.class, "left_lock");
        right_lock.setDirection(Servo.Direction.REVERSE);

        // Initialize the locks to a known state
        lock_state = release();
    }

    public boolean lock (){
        right_lock.setPosition(LOCK_POSITION_RIGHT);
        left_lock.setPosition(LOCK_POSITION_LEFT);

        lock_state = true;
        return lock_state;
    }

    public boolean release () {
        right_lock.setPosition(RELEASE_POSITION_RIGHT);
        left_lock.setPosition(RELEASE_POSITION_LEFT);
        lock_state = false;
        return lock_state;
    }

    public void toggleLocks () {
        if ( lock_state == true ) release();
        else lock();
    }

    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }

}
