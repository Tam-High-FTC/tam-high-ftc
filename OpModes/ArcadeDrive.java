package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Template: Arcade Drive", group = "Linear Opmode")
public class ArcadeDrive extends LinearOpMode {
    // Declare the hardware variables
    private DcMotor leftFrontMotor, rightFrontMotor;
    private DcMotor leftBackMotor, rightBackMotor;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as
        // parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");

        // One of the pairs of motors needs to be reversed
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the drive to press the Start button on the Driver Hub
        waitForStart();

        // Loop until the robot is stopped
        while (opModeIsActive()) {
            // Get the gamepad inputs
            // The y axis is vertical and "backwards",
            // so we negate it to make it positive when going forward
            float deltaY = -gamepad1.left_stick_y;
            float deltaX = gamepad1.left_stick_x;

            float left = deltaY + deltaX;
            float right = deltaY - deltaX;

            // Set the motor power
            leftFrontMotor.setPower(left);
            rightFrontMotor.setPower(right);
            leftBackMotor.setPower(left);
            rightBackMotor.setPower(right);
        }
    }
}
