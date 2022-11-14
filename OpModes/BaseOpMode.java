package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseOpMode extends LinearOpMode{
    protected DcMotorEx leftFront, rightFront;
    protected DcMotorEx leftBack, rightBack;
    protected BNO055IMU imu;
    protected DcMotorEx motorOne, motorTwo, motorThree;
    protected Servo clawServoRight, clawServoLeft; // right is 1, left is 2
    
    public static float TICKS_PER_ROTATION = 560f;
    
    public float rotateSpeed = 1f;
    
    public float targetRotation = 0f;
    public float targetX = 0f;
    public float targetY = 0f;
    public float currentDeltaX = 0f;
    public float currentDeltaY = 0f;
    public float currentX = 0f;
    public float currentY = 0f;
    
    public void setMotorRunToPosition(DcMotorEx motor){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);    
        motor.setVelocity(720f, AngleUnit.DEGREES);
        motor.setPower(1f);
    }
    
    public boolean isBusy(){
        return (leftFront.isBusy ()
        || rightFront.isBusy()
        || leftBack.isBusy()
        || rightBack.isBusy());
    }
    
    @Override
    public void runOpMode(){
        // Initialize the hardware variables. Note that the strings used here as
        // parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        clawServoRight = hardwareMap.get(Servo.class, "servo_one");
        clawServoLeft = hardwareMap.get(Servo.class, "servo_two");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        motorOne = hardwareMap.get(DcMotorEx.class, "lift_one");
        motorTwo = hardwareMap.get(DcMotorEx.class, "lift_two");
        motorThree = hardwareMap.get(DcMotorEx.class, "lift_three");
        
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

        imu.initialize(parameters);
        
        // One of the pairs of motors needs to be reversed
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        // rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        // START SETUP FOR LIFT SYSTEM
        // Initialize the hardware variables. Note that the strings used here as
        // parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        motorOne.setDirection(DcMotor.Direction.REVERSE);
        motorThree.setDirection(DcMotor.Direction.REVERSE);
        setMotorRunToPosition(motorOne);
        setMotorRunToPosition(motorTwo);
        setMotorRunToPosition(motorThree);
    }
}
