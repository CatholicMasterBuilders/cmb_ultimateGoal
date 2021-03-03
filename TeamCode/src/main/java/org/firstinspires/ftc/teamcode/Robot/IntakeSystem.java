package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSystem {
    // Data handles to ftc app
    Robot robot;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    LinearOpMode opMode;

    // Intake components
    DcMotor intake;
    Servo flicker;

    public final static int intake_arm_motor_type = 0; // 1 = motor with encoders, 0 = motor without encoders

    // Useful values for describing motor/servo positions
    private final static double SPINNER_SPIT = -1;
    private final static double SPINNER_SLURP = 1;
    private final static double SPINNER_HALF_SLURP = 0.4;
    private final static double SPINNER_OFF = 0;
    private boolean TOGGLE_STATE = false;

    // Constructors
    public IntakeSystem(Robot robot) {
        setRobot( robot );
    }

    // Initializers
    public void init() {
        // store the hardwareMap and telemetry for easy usage
        hardwareMap = getRobot().hardwareMap;
        telemetry = getRobot().telemetry;
        opMode = getRobot().opMode;

        intake = hardwareMap.get(DcMotor.class, "sweeper");

    }

    ///////////////////////////////////////////////
    // Functions for the intake wheel spinner
    ///////////////////////////////////////////////

//    // Clip the range of power values allowed for the intake arm
//    public double clipIntakeSpinnerValues ( double power ) {
//        return com.qualcomm.robotcore.util.Range.clip( power, SPINNER_SPIT, SPINNER_SLURP);
//    }

    // Set the intake spinner to an arbitrary power
    public double slurp () {
        setIntakePowers( SPINNER_SLURP );
        return SPINNER_SLURP;
    }

    public double getIntakePowers(){
        return intake.getPower();
    }

    public void setIntakePowers( double left_power) {
        intake.setPower( left_power );

    }

    public double half_slurp () {
        setIntakePowers( SPINNER_HALF_SLURP );
        return SPINNER_SLURP;
    }

    public double spit () {
        setIntakePowers( SPINNER_SPIT );
        return SPINNER_SPIT;
    }

    public double salivate () {
        setIntakePowers( SPINNER_OFF );
        return SPINNER_OFF;
    }

    public double setFlickerPosition (double position){

        flicker.setPosition(position);

        return position;
    }

    public void setFlickerUp () {
        setFlickerPosition(0.02);
    }

    public void setFlickerDown () {
        setFlickerPosition(0.36);
    }

    public void toggleFlicker () {
        if (TOGGLE_STATE) {
            setFlickerDown();
        } else {
            setFlickerUp();
        }
        TOGGLE_STATE = !TOGGLE_STATE;
    }

    public void blockInput() {
        setFlickerDown();
        TOGGLE_STATE = false;
    }

    // Getters and Setters
    public Robot getRobot() {
        return robot;
    }

    public void setRobot(Robot robot) {
        this.robot = robot;
    }
}
