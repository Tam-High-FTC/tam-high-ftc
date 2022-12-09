package org.firstinspires.ftc.teamcode;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This OpMode does basic image recognition
 * to identify a signal cone in Power Play,
 * and positions the robot in the appropriate signal zone.
 */
@Autonomous(name = "Main Autonomous", group = "Main")
public class AutonomousTensorFlow extends BaseOpMode {

    /**
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as
     * an "asset",
     * the OpMode must to load it using loadModelFromAsset(). However, if a team
     * generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be
     * loaded using loadModelFromFile()
     * Here we assume it's an Asset. Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20221023_141235.tflite";
    /**
     * Labels for the different symbols.
     * MUST match labels used when training the model.
     * SHOULD be unique for a given version of a symbol;
     * even for small changes, create a new label to avoid confusion.
     */
    private static final String[] LABELS = {
            "s1",
            "s4",
            "s5"
    };

    /**
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string
     * below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and
     * will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the
     * Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they
     * contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     * ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AUDmEnb/////AAABmcr7NyzGIUQ1mFRMqEx1d5oQVlzBkdCkCjPMZLGFJBLf2t8xedQZFzDv9euTbZk4wccepIt6tKKQ0PDzsl1ENV4LDk5FRo2FIbjRYeIyMFu9Qb3k/zTf00dQ8tcpooSyt+N5Z/h00oJ9QXpldq2YEyZyNA5zxDi4u6X9DWcx6VBBbTJaIkgqj91w7dg6EQGdRdXGa0IPFlVCXSCHwsIE3AHG3fb8r090UlSwfAGWBXrFj8bs8ZFMlIOyT3WQtJM8fz5bBt2eS6zFsact/lFNOtfCt0nkzncgsOnQJFEBJ6Xs703mdgddVNPjymQOhjSq9NWRZDneXf08l2flnXtLnhhtTZmpl/k8vBoy1Huaeq0i";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the
     * Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the
     * TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        super.runOpMode();

        setMotorRunToPosition(leftFrontMotor);
        setMotorRunToPosition(rightFrontMotor);
        setMotorRunToPosition(leftBackMotor);
        setMotorRunToPosition(rightBackMotor);

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we
        // create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow
         * annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a
            // lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or
            // 22").
            // If your target is at distance greater than 50 cm (20") you can increase the
            // magnification value
            // to artificially zoom in to the center of image. For best results, the
            // "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object
            // Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        
        boolean movedToTargetLocation = false;
        int selectedSymbol = -1;

        if (opModeIsActive()) {
            leftFrontMotor.setTargetPosition(-540);
            rightFrontMotor.setTargetPosition(-540);
            leftBackMotor.setTargetPosition(-540);
            rightBackMotor.setTargetPosition(-540);
            while (isBusy()) {
            }
            while (opModeIsActive() && selectedSymbol < 0) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available
                    // since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        float[] symbolConfidences = { 0f, 0f, 0f };

                        // step through the list of recognitions and display image position/size
                        // information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());
                            String label = recognition.getLabel();
                            float confidence = recognition.getConfidence();
                            for (int i = 0; i < 3; i++) {
                                if (label == LABELS[i]) {
                                    symbolConfidences[i] += confidence;
                                }
                            }

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", label, recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        }
                        float maxValue = 0.8f;
                        int i = 0;
                        for (float confidence : symbolConfidences) {
                            if (confidence > maxValue) {
                                maxValue = confidence;
                                selectedSymbol = i;
                            }
                            i += 1;
                        }
                        telemetry.addData("Symbol: ", selectedSymbol);
                        telemetry.update();
                    }

                }
            }
            leftFrontMotor.setTargetPosition(-1400);
            rightFrontMotor.setTargetPosition(-1400);
            leftBackMotor.setTargetPosition(-1400);
            rightBackMotor.setTargetPosition(-1400);
            while (isBusy()) {
            }
            switch (selectedSymbol) {
                case 0:
                    leftFrontMotor.setTargetPosition(0);
                    rightFrontMotor.setTargetPosition(-2800);
                    leftBackMotor.setTargetPosition(-2800);
                    rightBackMotor.setTargetPosition(0);
                    break;
                case 2:
                    leftFrontMotor.setTargetPosition(-2800);
                    rightFrontMotor.setTargetPosition(0);
                    leftBackMotor.setTargetPosition(0);
                    rightBackMotor.setTargetPosition(-2800);
                    break;
            }
        }
        addDrivetrainTelemetry();
        while(opModeIsActive()){}
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the
         * Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android
        // Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the
        // Robot Controller's FLASH.
        // tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}