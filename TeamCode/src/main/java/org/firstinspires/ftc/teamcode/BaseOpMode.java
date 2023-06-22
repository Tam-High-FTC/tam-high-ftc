package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;



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
    protected DcMotorEx liftMotorOne, liftMotorTwo;
    /** The rotation motor. */
    // protected DcMotorEx rotationMotor;
    /** The claw servos. */
    protected Servo clawServoRight, clawServoLeft;
    /** Dead Wheel Encoders */
    protected DcMotorEx deadWheelX, deadWheelY;

    /** Encoder ticks per rotation for 20:1 REV HD Hex Motors */
    public static final float DRIVE_TICKS_PER_ROTATION = 560f;
    /** Encoder value at initialization for lift motors. */
    public static final int LIFT_MIN_POSITION = 0;
    /** Encoder value at full extension for lift extension motors. */
    public static final int LIFT_MAX_POSITION = 4200;
    /** Encoder value at initialization for lift motors. */
    public static final int LIFT_MIN_ROTATION = -1600;
    /** Encoder value at full extension for lift extension motors. */
    public static final int LIFT_MAX_ROTATION = 1600;
    /**
     * Servo value when claw is fully open.
     * Not 1.0f because of limits on servo range.
     */
    public static final float CLAW_MAX_OPEN = 0.55f;
    /**
     * Ratio of encoder values at full extension for extension/retraction lift
     * motors
     */

    /** Claw float*/


    /** Scales changes to targetRotation. */
    public static final float ROTATION_SPEED = 1f;

    /** Claw rotations min and maxes*/

    /** Scales changes to targetLiftPosition. */
    public static final int LIFT_SPEED = 100;

    /** Scales changes to targetLiftPosition. */
    public static final float LIFT_MOTOR_MAX_VELOCITY = 3600f;

    // public static final float ROTATION_MOTOR_MAX_VELOCITY = 3600f;

    public static float UP = (float)Math.PI/2f;
    public static float LEFT = (float)Math.PI;
    public static float RIGHT = (float)0f;
    public static float DOWN = (float)-Math.PI/2f;

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
    public boolean inViewingPosition = false;
    public boolean caseSelected = false;
    public float targetRotation = 0f;

    public float currentX = 0f;
    public float currentY = 0f;

    /** Claw Rotation*/
    public int targetLiftRotation = 0;


    /** What x-axis position should the robot move towards? */
    public float targetX = 0f;
    /** What y-axis position should the robot move towards? */
    public float targetY = 0f;

    public float deltaX = 0f;
    public float deltaY = 0f;
    public float accumulatedDeltaX = 0;
    public float accumulatedDeltaY = 0;



    /** Configure the passed motor to use RUN_TO_POSITION mode. */
    public void setMotorRunToPosition(DcMotorEx motor, float maxVelocityDegrees){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setVelocity(360000f, AngleUnit.DEGREES);
        motor.setPower(1f);
    }

    /** Are any of the drivetrain motors currently busy? */
    public boolean isBusy() {
        return Math.abs(currentX - targetX) > 0.085 || Math.abs(currentY - targetY) > 0.085;
        // return (leftFrontMotor.isBusy()
        //         || rightFrontMotor.isBusy()
        //         || leftBackMotor.isBusy()
        //         || rightBackMotor.isBusy());
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
        liftMotorOne.setTargetPosition(this.targetLiftPosition);
        liftMotorTwo.setTargetPosition(this.targetLiftPosition);
    }

    /** Rotates the lift by the given number of encoder ticks. */
    // public void rotateLift(int positionDelta) {
    //     this.targetLiftRotation += positionDelta;
    //     rotateLiftToPosition(this.targetLiftRotation);
    // }

    /**
     * Sets this.targetLiftPosition within bounds,
     * and moves motors to target that position.
     */

    // public void rotateLiftToPosition(int targetLiftRotation) {
    //     this.targetLiftRotation = (int) Math.min(LIFT_MAX_ROTATION,
    //             Math.max(LIFT_MIN_ROTATION, targetLiftRotation));
    //     rotationMotor.setTargetPosition(this.targetLiftRotation);
    // }


    /** Update robot to a  position using odometry */
    public void updatePosition(){
        currentY = -deadWheelY.getCurrentPosition()/43838f;
        currentX = deadWheelX.getCurrentPosition()/43838f;
    }

    /** set robot to specific position in tile coordinates*/
    protected void setTargetPosition(float targetX, float targetY){
        this.targetX = targetX;
        this.targetY = targetY;
    }

    /** move robot to target position */
    public float[] getMovementMotorPowers(float magnitude, float direction){
        float[] motorPowers = {
                magnitude * (float) Math.sin(direction + Math.PI / 4),
                magnitude * (float) Math.sin(direction - Math.PI / 4),
                magnitude * (float) Math.sin(direction - Math.PI / 4),
                magnitude * (float) Math.sin(direction + Math.PI / 4),
        };
        return motorPowers;
    }
    protected void moveToTargetPosition(){
        moveToTargetPosition(1.0f, false);
    }
    /**
     * Move the robot to it's current targetX and targetY.
     * Uses this.currentX and this.currentY; DOES NOT UPDATE THEM
     * Ensure that you run updatePosition() earlier in the loop.
     */
    protected void moveToTargetPosition(float powerMax, boolean diagonal){
        // Proportional Coefficient
        float kP = 2.2f;
        float kI = 0.00f;

        float[] finalPowers = {0f,0f,0f,0f};
        deltaX = Math.abs(currentX - targetX);
        deltaY = Math.abs(currentY - targetY);
        accumulatedDeltaX += targetX - currentX;
        accumulatedDeltaY += targetY - currentY;
        float xTerm = deltaX * kP + Math.abs(accumulatedDeltaX) * kI;
        float yTerm = deltaY * kP + Math.abs(accumulatedDeltaY) * kI;

        if (diagonal){
            finalPowers = getMovementMotorPowers((float)Math.sqrt(yTerm * yTerm + xTerm + xTerm),(float) Math.atan2(yTerm, xTerm));
        } else if (deltaX <= deltaY) {
            if (deltaX < 0.10){ // change inequality as needed
                finalPowers = getMovementMotorPowers(yTerm,
                        targetY - currentY > 0
                                ? BaseOpMode.UP
                                : BaseOpMode.DOWN
                );
            } else {
                finalPowers = getMovementMotorPowers(xTerm,
                        targetX - currentX > 0
                                ? BaseOpMode.RIGHT
                                : BaseOpMode.LEFT
                );
            }
        }else if (deltaY < deltaX){
            if (deltaY < 0.10){ // change inequality as needed
                finalPowers = getMovementMotorPowers(xTerm,
                        targetX - currentX > 0
                                ? BaseOpMode.RIGHT
                                : BaseOpMode.LEFT
                );
            } else {
                finalPowers = getMovementMotorPowers(yTerm,
                        targetY - currentY > 0
                                ? BaseOpMode.UP
                                : BaseOpMode.DOWN
                );
            }


        }
        leftFrontMotor.setPower(Math.min(finalPowers[0], powerMax));
        leftBackMotor.setPower(Math.min(finalPowers[1], powerMax));
        rightFrontMotor.setPower(Math.min(finalPowers[2], powerMax));
        rightBackMotor.setPower(Math.min(finalPowers[3], powerMax));
    }



    /** Move claw to a specific position. 0 is closed, 1 is open. */
    public void moveClaw(float position) {
        position = Math.max(0, Math.min(CLAW_MAX_OPEN, position));
        clawServoRight.setPosition(1 - position);
        clawServoLeft.setPosition(position);
    }

    /** Open or close the claw. */
    public void moveClaw(boolean open) {
        clawServoRight.setPosition(open ? CLAW_MAX_OPEN : 0f);
        clawServoLeft.setPosition(open ? 0f : CLAW_MAX_OPEN);
    }

    /** Open the claw. */
    public void openClaw() {
        moveClaw(true);
    }

    /** Close the claw. */
    public void closeClaw() {
        moveClaw(false);
    }

    //* Add important info for Auto*/
    public void addAutoTelemetry() {
        telemetry.addData("Is Robot inViewingPosition?: ", inViewingPosition);
        telemetry.addData("Is robot 'busy'?: ", isBusy());
        telemetry.addData("Case Selected?: ",caseSelected);
        telemetry.addLine("ODOMETRY");
        telemetry.addData("Dead Wheel X: ", deadWheelX.getCurrentPosition());
        telemetry.addData("Dead Wheel Y: ", deadWheelY.getCurrentPosition());
        telemetry.addData("X position: ", currentX);
        telemetry.addData("Y position: ", currentY);
        telemetry.addData("Delta X: ", deltaX);
        telemetry.addData("Delta Y: ", deltaY);
        telemetry.addLine("DRIVETRAIN");
        telemetry.addData("LF", leftFrontMotor.getCurrentPosition());
        telemetry.addData("RF", rightFrontMotor.getCurrentPosition());
        telemetry.addData("LB", leftBackMotor.getCurrentPosition());
        telemetry.addData("RB", rightBackMotor.getCurrentPosition());
    }
    /** Add drive motor positions, etc., to telemetry data. */
    public void addDrivetrainTelemetry() {
        telemetry.addLine("DRIVETRAIN");
        telemetry.addData("LF", leftFrontMotor.getCurrentPosition());
        telemetry.addData("RF", rightFrontMotor.getCurrentPosition());
        telemetry.addData("LB", leftBackMotor.getCurrentPosition());
        telemetry.addData("RB", rightBackMotor.getCurrentPosition());
    }
    /** Add Odometry encode to telemetry */
    public void addOdometryTelemetry() {
        telemetry.addData("Dead Wheel X: ", deadWheelX.getCurrentPosition());
        telemetry.addData("Dead Wheel Y: ", deadWheelY.getCurrentPosition());
        telemetry.addData("X position: ", currentX);
        telemetry.addData("Y position: ", currentY);
        telemetry.addData("Delta X: ", deltaX);
        telemetry.addData("Delta Y: ", deltaY);
    }
    /** Add claw telemetry to telemetry data*/


    /** Add lift motor positions, etc., to telemetry data. */
    public void addLiftTelemetry() {
        telemetry.addData("Lift Position One: ", liftMotorOne.getCurrentPosition());
        telemetry.addData("Lift Position Two: ", liftMotorTwo.getCurrentPosition());
        telemetry.addData("Lift Target Position: ", targetLiftPosition);
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
        deadWheelX = hardwareMap.get(DcMotorEx.class, "left_front");
        deadWheelY = hardwareMap.get(DcMotorEx.class, "right_front");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        clawServoRight = hardwareMap.get(Servo.class, "servo_one");
        clawServoLeft = hardwareMap.get(Servo.class, "servo_two");
        // rotationMotor = hardwareMap.get(DcMotorEx.class, "motor_rotation");
        liftMotorOne = hardwareMap.get(DcMotorEx.class, "lift_one");
        liftMotorTwo = hardwareMap.get(DcMotorEx.class, "lift_two");


        // Configure the IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu.initialize(parameters);

        // SETUP DRIVETRAIN
        deadWheelX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        deadWheelY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        //rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
        //leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        //leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // Setup Dead Wheels

        //43838 is the number of encoder ticks per tile

        // START LIFT SYSTEM
        // liftMotorOne.setDirection(DcMotor.Direction.REVERSE);
        liftMotorTwo.setDirection(DcMotor.Direction.REVERSE);

        setMotorRunToPosition(liftMotorOne, LIFT_MOTOR_MAX_VELOCITY);
        setMotorRunToPosition(liftMotorTwo, LIFT_MOTOR_MAX_VELOCITY);


        // START CLAW SYSTEM
        // setMotorRunToPosition(rotationMotor, ROTATION_MOTOR_MAX_VELOCITY);
        // rotationMotor.setVelocityPIDFCoefficients(2f, 0.2f, 0f, 20f);


        // closeClaw();

    }
}