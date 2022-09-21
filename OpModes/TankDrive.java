package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Template: Tank Drive", group = "Linear Opmode")
public class TankDrive extends LinearOpMode {
    // Declare the hardware variables
    private DcMotor leftFront, rightFront;
    private DcMotor leftBack, rightBack;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as
        // parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        // One of the pairs of motors needs to be reversed
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DMotor.Direction.REVERSE);
        // Wait for the drive to press the Start button on the Driver Hub
        waitForStart();

        // Loop until the robot is stopped
        while (opModeIsActive()) {
            // Get the gamepad inputs
            // The y axis is vertical and "backwards",
            // so we negate it to make it positive when going forward
            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;

            // Set the motor power
            leftFront.setPower(left);
            rightFront.setPower(right);
            leftBack.setPower(left);
            rightBack.setPower(right);
        }
    }
}
