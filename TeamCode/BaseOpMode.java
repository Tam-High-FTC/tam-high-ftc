package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * BaseOpMode encapsulates the control logic for our robot,
 * and exposes a protected interface for children OpModes to use.
 */
public abstract class BaseOpMode extends LinearOpMode {
    /** The drivetrain motors. */
    protected DcMotorEx leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
    /**
     * The internal Inertial Measurement Unit.
     * Note that this chip is present on the old Expansion Hubs,
     * not on our new Control Hubs.
     * Waiting on SDK update for new chip. (2022-11-15)
     */
    protected BNO055IMU imu;
    /** The lift motors. */
    protected DcMotorEx liftMotorOne, liftMotorTwo, liftMotorThree;
    /** The claw servos. */
    protected Servo clawServoRight, clawServoLeft;

    /** Encoder ticks per rotation for 20:1 REV HD Hex Motors */
    public static final float DRIVE_TICKS_PER_ROTATION = 560f;

    /** Encoder value at initialization for lift motors. */
    public static final int LIFT_MIN_POSITION = 0;
    /** Encoder value at full extension for lift extension motors. */
    public static final int LIFT_MAX_POSITION = 1550;
    /**
     * Ratio of encoder values at full extension for extension/retraction lift
     * motors
     */
    public static final double LIFT_EXTEND_RETRACT_RATIO = 1750 / 1500;

    /** Scales changes to targetRotation. */
    public static final float ROTATION_SPEED = 1f;

    /** Scales changes to targetLiftPosition. */
    public static final float LIFT_SPEED = 1f;

    /**
     * What position should the lift go towards?
     * Encoder ticks
     * LIFT_MIN_POSITION to LIFT_MAX_POSITION
     */
    public int targetLiftPosition = 0;

    /**
     * What rotation value should the robot move towards?
     * Degrees
     * -179 to 179.
     */
    public float targetRotation = 0f;
    /** What x-axis position should the robot move towards? */
    public float targetX = 0f;
    /** What y-axis position should the robot move towards? */
    public float targetY = 0f;
    public float currentDeltaX = 0f;
    public float currentDeltaY = 0f;
    public float currentX = 0f;
    public float currentY = 0f;

    /** Configure the passed motor to use RUN_TO_POSITION mode. */
    public void setMotorRunToPosition(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(720f, AngleUnit.DEGREES);
        motor.setPower(1f);
    }

    /** Are any of the drivetrain motors currently busy? */
    public boolean isBusy() {
        return (leftFrontMotor.isBusy()
                || rightFrontMotor.isBusy()
                || leftBackMotor.isBusy()
                || rightBackMotor.isBusy());
    }

    /** Moves the lift by the given number of encoder ticks. */
    public void moveLift(int positionDelta) {
        this.targetLiftPosition += positionDelta;
        moveLiftToPosition(this.targetLiftPosition);
    }

    /**
     * Sets this.targetLiftPosition within bounds,
     * and moves motors to target that position.
     */
    public void moveLiftToPosition(int targetLiftPosition) {
        this.targetLiftPosition = (int) Math.min(LIFT_MAX_POSITION,
                Math.max(LIFT_MIN_POSITION, targetLiftPosition));
        liftMotorOne.setTargetPosition(targetLiftPosition);
        liftMotorTwo.setTargetPosition(targetLiftPosition);
        liftMotorThree
                .setTargetPosition(-(int) Math.floor((double) targetLiftPosition * LIFT_EXTEND_RETRACT_RATIO));
    }

    /** Add lift motor posititions, etc., to telemetry data. */
    public void addLiftTelemetry() {
        telemetry.addData("Lift Position One: ", liftMotorOne.getCurrentPosition());
        telemetry.addData("Lift Position Two: ", liftMotorTwo.getCurrentPosition());
        telemetry.addData("Lift Position Three: ", liftMotorThree.getCurrentPosition());
        telemetry.addData("Lift Target Position: ", targetLiftPosition);
    }

    /** Move claw to a specific position. 0 is closed, 1 is open. */
    public void moveClaw(float position) {
        position = Math.min(0, Math.max(1, position));
        clawServoRight.setPosition(position);
        clawServoLeft.setPosition(1 - position);
    }

    /** Open or close the claw. */
    public void moveClaw(bool open) {
        clawServoRight.setPosition(open ? 1 : 0);
        clawServoLeft.setPosition(open ? 0 : 1);
    }

    /** Open the claw. */
    public void openClaw() {
        moveClaw(true);
    }

    /** Close the claw. */
    public void closeClaw() {
        moveClaw(false);
    }

    @Override
    public void runOpMode() {
        /**
         * Initialize the hardware variables. Note that the strings used here as
         * parameters to 'get' must correspond to the names assigned during the robot
         * configuration on the Driver Hub.
         */
        leftFrontMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotorEx.class, "right_back");
        clawServoRight = hardwareMap.get(Servo.class, "servo_one");
        clawServoLeft = hardwareMap.get(Servo.class, "servo_two");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "lift_one");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "lift_two");
        liftMotorThree = hardwareMap.get(DcMotorEx.class, "lift_three");

        // Configure the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu.initialize(parameters);

        // SETUP DRIVETRAIN
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        // rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // START LIFT SYSTEM
        liftMotorOne.setDirection(DcMotor.Direction.REVERSE);
        liftMotorThree.setDirection(DcMotor.Direction.REVERSE);
        setMotorRunToPosition(liftMotorOne);
        setMotorRunToPosition(liftMotorTwo);
        setMotorRunToPosition(liftMotorThree);
    }
}
