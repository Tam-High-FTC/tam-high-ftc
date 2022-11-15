package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

// We will need some operators from this library
import java.lang.Math;

@TeleOp(name = "Claw", group = "Testing")
public class ClawServoTest extends LinearOpMode {
    // Declare the hardware variables
    private Servo servoOne, servoTwo;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as
        // parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        servoOne = hardwareMap.get(Servo.class, "servo_one");
        servoTwo = hardwareMap.get(Servo.class, "servo_two");

        // Wait for the drive to press the Start button on the Driver Hub
        waitForStart();

        // Loop until the robot is stopped
        while (opModeIsActive()) {
            float clawOpen = gamepad1.left_trigger;

            servoOne.setPosition(clawOpen);
            servoTwo.setPosition(1 - clawOpen);

            telemetry.update();
        }
    }
}