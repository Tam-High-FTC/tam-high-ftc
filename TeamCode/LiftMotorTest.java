package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

// We will need some operators from this library
import java.lang.Math;

@TeleOp(name = "Test: Lift Motor Test", group = "Linear Opmode")
public class LiftMotorTest extends LinearOpMode {
    // Declare the hardware variables
    private DcMotor motorOne, motorTwo;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as
        // parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        motorOne = hardwareMap.get(DcMotor.class, "motor_one");
        motorTwo = hardwareMap.get(DcMotor.class, "motor_two");

        // Wait for the drive to press the Start button on the Driver Hub
        waitForStart();

        // Loop until the robot is stopped
        while (opModeIsActive()) {
            boolean motorUp = gamepad1.dpad_up;
            boolean motorDown = gamepad1.dpad_down;
            // First, we need to split the translation vector into a direction and a
            // magnitude.
            // The direction is the direction to move
            float motorPower = 0;
            if (motorUp) {
                motorPower = 1;
            } else if (motorDown) {
                motorPower = -1;
            }

            motorOne.setPower(motorPower);
            motorTwo.setPower(motorPower);

            telemetry.update();
        }
    }
}