package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PlacingSystem {
    Robot robot;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    LinearOpMode opMode;

    //Motors
    public DcMotor thingy;
    Servo dingusKahn;

    //Variables
    private final int COUNTS_PER_REV = 1484;
    private final int COUNTS_PER_DEGREE = 1484/360;
    private int target_position = 0;
    boolean isGrabberClosed = false;


    // Constructors
    public PlacingSystem(Robot robot) {
        setRobot(robot);
    }

    // Initializers
    public void init() {
        // store the hardwareMap and telemetry for easy usage
        hardwareMap = getRobot().hardwareMap;
        telemetry = getRobot().telemetry;

        thingy = hardwareMap.get(DcMotor.class, "thingy");
        dingusKahn = hardwareMap.get(Servo.class, "dingusKahn");

        thingy.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        thingy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thingy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        thingy.setDirection(DcMotorSimple.Direction.REVERSE);

        thingy.setPower(0);
    }


    public boolean placeGoal (boolean retractArm){
        target_position = COUNTS_PER_DEGREE * 170;

        thingy.setTargetPosition(target_position);

        thingy.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        thingy.setPower(-0.2);

        while (thingy.getPower() != 0.0) {
            if (thingy.isBusy() == false) {
                thingy.setPower(0);
            }
        }
        openDingusKahn();

        target_position = 0;

        thingy.setTargetPosition(target_position);

        thingy.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        thingy.setPower(0.5);

        while (thingy.getPower() != 0.0) {
            if (thingy.isBusy() == false) {
                thingy.setPower(0);
            }
        }
        return true;
    }

    public boolean placeGoal (){
        target_position = COUNTS_PER_DEGREE * 170;

        thingy.setTargetPosition(target_position);

        thingy.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        thingy.setPower(-0.2);

        while (thingy.getPower() != 0.0) {
            if (thingy.isBusy() == false) {
                thingy.setPower(0);
            }
        }
        openDingusKahn();

        return true;
    }

    public boolean retractArm (){

        target_position = 0;

        thingy.setTargetPosition(target_position);

        thingy.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        thingy.setPower(0.5);

        while (thingy.getPower() != 0.0) {
            if (thingy.isBusy() == false) {
                thingy.setPower(0);
            }
        }
        return true;

    }

    public boolean setArmPower (double power){


        if (power > 0)
            target_position = COUNTS_PER_DEGREE * 155;

        else if (power >= -0.1 && power <= 0.1) {
            target_position = thingy.getCurrentPosition();
        }

        else if (power < 0)
            target_position = COUNTS_PER_DEGREE * 69;

        thingy.setTargetPosition(target_position);

        thingy.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        if (!thingy.isBusy()) {
            thingy.setPower(0);
            return true;
        }

        else {
            thingy.setPower(power * 0.5);
            return false;
        }

    }

    public boolean closeDingusKahn() {

        dingusKahn.setPosition(0.58);
        isGrabberClosed = true;

        return true;
    }

    public boolean openDingusKahn() {

        dingusKahn.setPosition(0.94);

        isGrabberClosed = false;

        return false;
    }

    public void toggleDingusKahn(){
        if (! isGrabberClosed){
            closeDingusKahn();
        }
        else {
            openDingusKahn();
        }
    }

    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }


    }
