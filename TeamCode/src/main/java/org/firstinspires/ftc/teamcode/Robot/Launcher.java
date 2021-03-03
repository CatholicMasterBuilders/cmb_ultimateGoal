package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher {

    private Robot robot;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private LinearOpMode opMode;

    private DcMotor chongus;
    private CRServo setter;
    private Servo aimer;
    private Servo scorpion_tail;
    private Servo dingusKahn;

    public final double AIMER_DOWN_POS = 0.08;

    private double aimerPos = 1;

    public Launcher (Robot robot){
        setRobot(robot);

        hardwareMap = getRobot().hardwareMap;
        telemetry = getRobot().telemetry;
        opMode = getRobot().opMode;

        chongus = hardwareMap.get(DcMotor.class, "chongus");
        setter = hardwareMap.get(CRServo.class, "setter");
        aimer = hardwareMap.get(Servo.class, "aimer");
        scorpion_tail = hardwareMap.get(Servo.class, "scorpion_tail");
        dingusKahn = hardwareMap.get(Servo.class, "dingusKahn");

        chongus.setDirection(DcMotorSimple.Direction.REVERSE);
        chongus.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Stops the setter to stop giving rings to chongus
     */
    public void hitFourYouHaveGoneTooFar(){
        setter.setPower(0);
    }

    /**
     * Stops chongus to halt launching of rings
     */
    public void holyHandGranadeManuel(){ chongus.setPower(0); }

    /**
     * Reverses chongus at lower power and the setter to bring rings up form intake
     */
    public void pullHolyGranadePin () {
        setter.setPower(-1);
        chongus.setPower(-0.5);
    }

    /**
     * Sets chonngus to full power to launch rings
     */
    public void one (){
        chongus.setPower(0.90);
    }

    public void one (double power) {
        chongus.setPower(power);
    }

    /**
     * sets the position of the launcher ramp to the specified position
     * @param pos
     */
    public void two (double pos){
        aimerPos = pos;

        aimer.setPosition(aimerPos);
    }

    /**
     * Sets the launcher to the high goal position
     */
    public void two (){
        aimer.setPosition(0.431);
    }

    /**
     * Sets the setter to full power to give a right to chongus to launch
     */
    public void five_ThreeSir_Three (){
        setter.setPower(1);
    }

    public void setScorpion_tail (double pos ) { scorpion_tail.setPosition(pos); }

    public void setScorpion_tail () { scorpion_tail.setPosition(0.3); }

    public void openDingusKahn () {
        dingusKahn.setPosition(0.27);
    }

    public void closeDingusKahn (){
        dingusKahn.setPosition(0.4);
    }



    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }
}
