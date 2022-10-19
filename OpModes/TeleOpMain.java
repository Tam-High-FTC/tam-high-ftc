package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BHI260IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Gyroscope;

// We will need some operators from this library
import java.lang.Math;

@TeleOp(name = "Main TeleOp Mode", group = "Linear Opmode")
public class TeleOpMain extends LinearOpMode {
    // Declare the hardware variables
    private DcMotorEx leftFront, rightFront;
    private DcMotorEx leftBack, rightBack;
    private BHI260IMU imu;
    
    public static float TICKS_PER_ROTATION = 560f;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as
        // parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize();

        // One of the pairs of motors needs to be reversed
        // rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        

        // Wait for the drive to press the Start button on the Driver Hub
        waitForStart();
        
        // PIDCoefficients pidOrig = leftFront.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Loop until the robot is stopped
        while (opModeIsActive()) {
            // The left joystick controls the translation of the robot,
            // while the right joystick controls the rotation.

            // Get the gamepad inputs
            // The y axis is vertical and "backwards",
            // so we negate it to make it positive when going forward
            // ^ +y
            // |
            // <- ->
            // -x | +x
            // v -y
            float sensitivity = 1f;
            float rotationsPerSecond = 300f;
            if (gamepad1.left_bumper){
                sensitivity = 2f;
            }
            float deltaY = -gamepad1.left_stick_y;
            float deltaX = gamepad1.left_stick_x;
            // Rotate by moving right stick left-right
            float rotation = gamepad1.right_stick_x;
            // First, we need to split the translation vector into a direction and a
            // magnitude.
            // The direction is the direction to move
            float direction = (float)Math.atan2(deltaY, deltaX);
            telemetry.addData("Direction: %f", direction);
            if (direction > 0){
                direction = (float)Math.PI - direction;
            } else {
                direction = -direction - (float)Math.PI;
            }
            // The magnitude is how fast to move
            float magnitude = (float)Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
            float speed = rotationsPerSecond
                * BasicMecanumDrive.TICKS_PER_ROTATION;
            
            // TODO Explain these calculations
            float leftFrontPower = magnitude * (float)Math.sin(direction + Math.PI / 4) + rotation;
            float leftBackPower = magnitude * (float)Math.sin(direction - Math.PI / 4) + rotation;
            float rightFrontPower = magnitude * (float)Math.sin(direction - Math.PI / 4) - rotation;
            float rightBackPower = magnitude * (float)Math.sin(direction + Math.PI / 4) - rotation;
            telemetry.addData("leftFrontPower: %f", leftFrontPower);
            telemetry.addData("leftBackPower: %f", leftBackPower);
            telemetry.addData("rightFrontPower: %f", rightFrontPower);
            telemetry.addData("rightBackPower: %f", rightBackPower);

            // All of the values must be scaled to be within [-1,1]
            // First, find the highest value
            float maxPower = sensitivity * Math.max(
                Math.abs(leftFrontPower),
                    Math.max(
                        Math.abs(rightBackPower),
                        Math.max(
                            Math.abs(leftBackPower),
                            Math.abs(rightBackPower))));
            // Then divide all the powers by that value.
            leftFrontPower = leftFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
            rightBackPower = rightBackPower / maxPower;

            // Set the motor power
            leftFront.setVelocity(speed * leftFrontPower);
            leftBack.setVelocity(speed * leftBackPower);
            rightFront.setVelocity(speed * rightFrontPower);
            rightBack.setVelocity(speed *rightBackPower);
            
            telemetry.addData("Encoder", leftFront.getCurrentPosition());
            // imu.get
            Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("%f", currentAngles.firstAngle);
            telemetry.addData("%f", currentAngles.secondAngle);
            telemetry.addData("%f", currentAngles.thirdAngle);
            // telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.04f",
            //     pidOrig.p, pidOrig.i, pidOrig.d);
            telemetry.update();
        }
    }
}