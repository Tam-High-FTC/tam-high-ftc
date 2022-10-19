package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// We will need some operators from this library
import java.lang.Math;

@TeleOp(name = "Test: Lift Motor Test", group = "Linear Opmode")
public class LiftMotorTest extends LinearOpMode {
    // Declare the hardware variables
    private DcMotorEx motorOne, motorTwo;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here as
        // parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        motorOne = hardwareMap.get(DcMotorEx.class, "lift_one");
        motorTwo = hardwareMap.get(DcMotorEx.class, "lift_two");
        
        motorOne.setDirection(DcMotor.Direction.REVERSE);
        
        motorOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        motorOne.setVelocity(180f, AngleUnit.DEGREES);
        motorTwo.setVelocity(180f, AngleUnit.DEGREES);
        
        motorOne.setPower(0.5f);
        motorTwo.setPower(0.5f);
        
        motorOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        int minPosition = 0;
        int maxPosition = 1550; // 6 turns at 288 ticks per turn
        int targetPosition = 0;
        int speed = 2;
        
        // Wait for the drive to press the Start button on the Driver Hub
        waitForStart();

        // Loop until the robot is stopped
        while (opModeIsActive()) {
            boolean motorUp = gamepad1.dpad_up;
            boolean motorDown = gamepad1.dpad_down;
            // First, we need to split the translation vector into a direction and a
            // magnitude.
            // The direction is the direction to move
            if(motorUp){
                targetPosition += speed;
            } else if (motorDown){
                targetPosition -= speed;
            }
            targetPosition = (int)
                Math.min(maxPosition,
                Math.max(minPosition, targetPosition)
                );
            
            motorOne.setTargetPosition(targetPosition);
            motorTwo.setTargetPosition(targetPosition);
            telemetry.addData("Current Position One: ", motorOne.getCurrentPosition());
            telemetry.addData("Current Position Two: ", motorTwo.getCurrentPosition());
            telemetry.addData("Target Position: ", targetPosition);
            telemetry.update();
        }
    }
}