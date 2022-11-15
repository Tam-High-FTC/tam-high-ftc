package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Template: Basic Mecanum Drive", group = "Linear Opmode")
public class TeleOpMain extends BaseOpMode {

    @Override
    public void runOpMode() {
        super.runOpMode();

        int minPosition = 0;
        int maxPosition = 1550; // 6 turns at 288 ticks per turn
        int maxPositionPull = 1750;
        int targetPosition = 0;
        int liftSpeed = 10;
        double upDownRatio = (maxPositionPull / maxPosition); // Ratio of maxPositionPull to maxPosition
        // END SETUP FOR LIFT SYSTEM

        // Wait for the drive to press the Start button on the Driver Hub
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
        // PIDCoefficients pidOrig =
        // leftFront.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Loop until the robot is stopped
        long lastTime = System.nanoTime() / 1000000;
        while (opModeIsActive()) {
            long time = System.nanoTime() / 1000000;
            int deltaTime = (int) (time - lastTime);
            lastTime = time;

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
            if (gamepad1.left_bumper) {
                sensitivity = 1f;
            }
            float deltaY = -gamepad1.left_stick_y;
            float deltaX = gamepad1.left_stick_x;

            // Old rotation used as new rotation is bugged

            float oldRotation = gamepad1.right_stick_x;

            // Rotate by moving right stick left-right

            float playerRotation = gamepad1.right_stick_x;
            targetRotation -= playerRotation * rotateSpeed;

            // First, we need to split the translation vector into a direction and a
            // magnitude.
            // The direction is the direction to move
            float direction = (float) Math.atan2(deltaY, deltaX);
            telemetry.addData("Direction: %f", direction);
            if (direction > 0) {
                direction = (float) Math.PI - direction;
            } else {
                direction = -direction - (float) Math.PI;
            }
            // The magnitude is how fast to move
            float magnitude = (float) Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
            float speed = rotationsPerSecond
                    * BasicMecanumDrive.TICKS_PER_ROTATION;

            // Amend rotation to correct for drift
            float currentRotation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                    AngleUnit.DEGREES).firstAngle;
            float remappedTargetRotation = targetRotation + 180f;
            float inRangeRemappedTargetRotation = (remappedTargetRotation % 360f + 360f) % 360f;
            targetRotation = inRangeRemappedTargetRotation - 180f;
            float rotationStep = 45f;
            float steppedTargetRotation = ((float) Math.round(targetRotation / rotationStep) * rotationStep);
            if (steppedTargetRotation == -180f)
                steppedTargetRotation = -179f;
            float rotationDelta = steppedTargetRotation - currentRotation;
            if (rotationDelta > 180f)
                rotationDelta -= 360f;
            if (rotationDelta < -180f)
                rotationDelta += 360f;
            float finalRotationPower = rotationDelta / -3600f;
            telemetry.addLine("Rotation");
            telemetry.addData("Current: ", currentRotation);
            telemetry.addData("Target: ", targetRotation);
            telemetry.addData("Remapped: ", inRangeRemappedTargetRotation);
            telemetry.addData("Stepped: ", steppedTargetRotation);
            telemetry.addData("Final Power", finalRotationPower);

            // TARGET POSITION CALCS

            Position position = imu.getPosition();
            telemetry.addData("Position: ", position);
            Acceleration acceleration = imu.getAcceleration();
            telemetry.addData("Acceleration: ", acceleration);
            float xAccel = 0f;
            if (Math.abs(acceleration.xAccel) > 0.5f) {
                xAccel = (float) acceleration.xAccel;
            }
            ;
            float yAccel = 0f; // Grabs raw Z due to orientation of hub
            if (Math.abs(acceleration.zAccel) > 0.5f) {
                yAccel = (float) acceleration.zAccel;
            }
            ;
            currentDeltaX += xAccel * deltaTime / 1000;
            currentDeltaY += yAccel * deltaTime / 1000;
            currentX += currentDeltaX * deltaTime / 1000;
            currentY += currentDeltaY * deltaTime / 1000;
            telemetry.addData("Current Delta X: ", currentDeltaX);
            telemetry.addData("Current Delta Y: ", currentDeltaY);
            telemetry.addData("Current X: ", currentX);
            telemetry.addData("Current Y: ", currentY);

            // END TARGET POSITION CALCS

            // TODO Explain these calculations
            float leftFrontPower = magnitude * (float) Math.sin(direction + Math.PI / 4) + oldRotation;
            float leftBackPower = magnitude * (float) Math.sin(direction - Math.PI / 4) + oldRotation;
            float rightFrontPower = magnitude * (float) Math.sin(direction - Math.PI / 4) - oldRotation;
            float rightBackPower = magnitude * (float) Math.sin(direction + Math.PI / 4) - oldRotation;

            // All of the values must be scaled to be within [-1,1]
            // First, find the highest value
            float maxPower = sensitivity *
                    Math.max(1f, Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightBackPower),
                            Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower)))));

            // Then divide all the powers by that value.
            leftFrontPower = leftFrontPower / maxPower;
            leftBackPower = leftBackPower / maxPower;
            rightFrontPower = rightFrontPower / maxPower;
            rightBackPower = rightBackPower / maxPower;
            telemetry.addData("maxPower: ", maxPower);
            telemetry.addData("leftFrontPower: ", leftFrontPower);
            telemetry.addData("leftBackPower: ", leftBackPower);
            telemetry.addData("rightFrontPower: ", rightFrontPower);
            telemetry.addData("rightBackPower: ", rightBackPower);

            // Set the motor power
            leftFront.setVelocity(speed * leftFrontPower);
            leftBack.setVelocity(speed * leftBackPower);
            rightFront.setVelocity(speed * rightFrontPower);
            rightBack.setVelocity(speed * rightBackPower);

            // START LIFT
            boolean motorUp = gamepad1.dpad_up;
            boolean motorDown = gamepad1.dpad_down;
            if (motorUp) {
                targetPosition += liftSpeed;
            } else if (motorDown) {
                targetPosition -= liftSpeed;
            }
            // Clamp targetPosition to min/maxPosition
            targetPosition = (int) Math.min(maxPosition,
                    Math.max(minPosition, targetPosition));
            motorOne.setTargetPosition(targetPosition);
            motorTwo.setTargetPosition(targetPosition);
            motorThree.setTargetPosition(-(int) Math.floor((double) targetPosition * upDownRatio));
            telemetry.addData("Lift Position One: ", motorOne.getCurrentPosition());
            telemetry.addData("Lift Position Two: ", motorTwo.getCurrentPosition());
            telemetry.addData("Lift Position Three: ", motorThree.getCurrentPosition());
            telemetry.addData("Lift Target Position: ", targetPosition);
            // END LIFT

            // START CLAW
            float clawOpen = gamepad1.left_trigger;
            clawServoRight.setPosition(clawOpen);
            clawServoLeft.setPosition(1 - clawOpen);
            // END CLAW

            telemetry.addData("LF", leftFront.getCurrentPosition());
            telemetry.addData("RF", rightFront.getCurrentPosition());
            telemetry.addData("LB", leftBack.getCurrentPosition());
            telemetry.addData("RB", rightBack.getCurrentPosition());
            telemetry.update();
        }
    }
}
