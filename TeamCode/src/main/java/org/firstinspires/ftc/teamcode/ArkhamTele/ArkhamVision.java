package org.firstinspires.ftc.teamcode.ArkhamTele;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class ArkhamVision {
    public static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    public static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    public static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    public static final String VUFORIA_KEY = " AeGzv1L/////AAAAGVITNWG9l0vpn4p4pvlP/jcpnVwgp6LntDGpzlU29IDkrI1lgYfUDb1aMdcWejc2SPKfy8OAGx1jdELMNfkwxKrIyrAt0Udc6N5gERLf+5804xm3ZRG5yh7nY+ttX0UL8FZLEajtXW2XGLzlFZEuB2yHrvjFYFheO1tCl6wuGyQZdVSaMuJyeIYeyvTYAJNA38/ZsH8oYjKl/VIIV7KTx8CNAcevdBu6VIrtHTXEXbTVIb5IahynsMvY1vESupkdoPAqWwbmdnnRwaCOSD1HccaqxXnFGi595AZ5MhJb2CiZoiDjV5HnayBOXfeiK/PU2nyrCANHM1VkN6zKhA0i8rGKNwclST6uqAfosEmU62Xf ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;

    HardwareMap hwMap = null;

    public ArkhamVision() {
    }

    public void initVision(HardwareMap ahwMap) {
        hwMap = ahwMap;
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.


        /**
         * Initialize the Tensor Flow Object Detection engine.
         */

        int tfodMonitorViewId = ahwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", ahwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        tfod.activate();


    }



    public void Disable() {
            tfod.shutdown();
    }

}






