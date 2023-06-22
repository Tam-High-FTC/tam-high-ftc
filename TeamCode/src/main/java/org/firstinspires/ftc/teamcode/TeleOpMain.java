package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The main TeleOp OpMode for the 2022-2023 POWER PLAY game.
 */
@TeleOp(name = "Main TeleOp", group = "Main")
public class TeleOpMain extends BaseOpMode {

    @Override
    public void runOpMode() {
        super.runOpMode();

        // Wait for the driver to press the Start button on the Driver Hub
        waitForStart();

        // Start position/acceleration tracking for IMU
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);

        while (opModeIsActive()) {

            // The left joystick controls the translation of the robot,
            // while the right joystick controls the rotation.

            float inputSensitivity = (0.6f * gamepad1.left_trigger) + 0.4f;
            float maxDrivetrainRotationsPerSecond = 10f;

            // Get the gamepad inputs
            // The y axis is vertical and "backwards",
            // so we negate it to make it positive when going forward
            float deltaYInput = -gamepad1.left_stick_y;
            float deltaXInput = -gamepad1.left_stick_x;
            float rotationInput = gamepad1.right_stick_x;

            // First, we need to split the translation vector
            // into a direction and a magnitude.
            // The direction is the direction to move in radians
            float direction = (float) Math.atan2(deltaYInput, deltaXInput);
            // The magnitude is how fast to move
            float magnitude = (float) Math.sqrt(Math.pow(deltaXInput, 2) + Math.pow(deltaYInput, 2));
            telemetry.addData("Direction: %f", direction);
            // The direction must be rotated
            if (direction > 0) {
                direction = (float) Math.PI - direction;
            } else {
                direction = -direction - (float) Math.PI;
            }
            float leftFrontPower = magnitude * (float) Math.sin(direction + Math.PI / 4) + rotationInput;
            float leftBackPower = magnitude * (float) Math.sin(direction - Math.PI / 4) + rotationInput;
            float rightFrontPower = magnitude * (float) Math.sin(direction - Math.PI / 4) - rotationInput;
            float rightBackPower = magnitude * (float) Math.sin(direction + Math.PI / 4) - rotationInput;

            // All of the values must be scaled to be within [-1,1]
            // First, find the highest value
            float maxPower = Math.max(1f, Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightBackPower),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))))) / inputSensitivity;

            telemetry.addData("Max Power: ", maxPower);
            // Then divide all the powers by that value.
            leftFrontPower = leftFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
            rightBackPower = rightBackPower / maxPower;

            telemetry.addData("leftFrontPower: ", leftFrontPower);
            telemetry.addData("leftBackPower: ", leftBackPower);
            telemetry.addData("rightFrontPower: ", rightFrontPower);
            telemetry.addData("rightBackPower: ", rightBackPower);
            // Set the motor velocity
            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightFrontMotor.setPower(rightFrontPower);
            rightBackMotor.setPower(rightBackPower);

            // START LIFT
            boolean moveLiftFully = gamepad1.left_bumper;
            boolean motorUpInput = gamepad1.dpad_up;
            boolean motorDownInput = gamepad1.dpad_down;
            // Prevent movement if both or neither are pressed.
            if (motorUpInput != motorDownInput) {
                if(moveLiftFully){
                    if (motorUpInput) {
                        moveLiftToPosition(LIFT_MAX_POSITION);
                    }
                    if (motorDownInput) {
                        moveLiftToPosition(LIFT_MIN_POSITION);
                    }
                }else{
                    if (motorUpInput) {
                        moveLift(LIFT_SPEED);
                    }
                    if (motorDownInput) {
                        moveLift(-LIFT_SPEED);
                    }
                }
            } else {
                moveLiftToPosition(targetLiftPosition);
            }
            addLiftTelemetry();
            // ADD ODOMETRY TELEMETRY
            addOdometryTelemetry();
            // END LIFT

            // START CLAW ROTATION
            // float liftRotationInput = gamepad2.left_stick_x;
            // if (liftMotorOne.getCurrentPosition() > 1500){
            //     rotateLift(-(int)Math.round(liftRotationInput * 20));
            // }
            // boolean forwardOverride = gamepad1.y;
            // if(forwardOverride){
            //     rotateLiftToPosition(0);
            // }
            // telemetry.addData("Rotation Input: ", liftRotationInput);
            // telemetry.addData("Rotation Motor: ", rotationMotor.getCurrentPosition());

            // END CLAW ROTATION

            // START CLAW
            float clawOpen = gamepad1.right_trigger;
            moveClaw(clawOpen);

            // END CLAW
            telemetry.addData("Claw Value: ", clawOpen);
            // addDrivetrainTelemetry();
            updatePosition();
            telemetry.update();
        }
    }
}
