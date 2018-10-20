package org.firstinspires.ftc.teamcode.Auto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Hardware.BB13HW;
import org.firstinspires.ftc.teamcode.Hardware.BB13Sensors;

import java.util.Locale;

import static org.firstinspires.ftc.teamcode.Auto.AutoBot.State.Sample;
import static org.firstinspires.ftc.teamcode.Auto.AutoBot.State.Sample;

//...
/**
 * Created by Brady on 1/3/2018.
 */

@Autonomous(name = "Sample")


public class Sample extends OpMode {

    BB13HW robot = new BB13HW();
    BB13Sensors sensors = new BB13Sensors();
    private SamplingOrderDetector detector;

    enum State {
        Start,Forward,left,center,right, Sample, Stop}

    State state;
    ElapsedTime time;

    @Override
    public void init() {
        robot.init(hardwareMap);
        sensors.initSensors(hardwareMap);
        state = State.Start;
        time = new ElapsedTime();
        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");

        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.2; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
    }

    @Override
    public void loop() {
        double CurrentTime = time.time();
        telemetry.addData("time", CurrentTime);
        double gyroangle;
        sensors.angles = sensors.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gyroangle = Double.parseDouble(formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
        telemetry.addData("Heading", formatAngle(sensors.angles.angleUnit, sensors.angles.firstAngle));
        telemetry.addData("Current Order" , detector.getCurrentOrder().toString()); // The current result for the frame
        telemetry.addData("Last Order" , detector.getLastOrder().toString()); // The last known result
        switch (state) {

            case Start:
                robot.Kill();
                if (robot.IsBusy()) {
                    state = State.Sample;
                    time.reset();
                }
                break;

            case Sample:
                if(detector.getLastOrder().equals(SamplingOrderDetector.GoldLocation.LEFT)&&CurrentTime>=2){
                    state = State.left;
                }
                if(detector.getLastOrder().equals(SamplingOrderDetector.GoldLocation.CENTER)&&CurrentTime>=2){
                    state = State.center;
                }
                if(detector.getLastOrder().equals(SamplingOrderDetector.GoldLocation.RIGHT)&&CurrentTime>=2){
                    state = State.right;
                }
                else {robot.Kill();}
                break;

            case left:
                robot.TurnAbsoulte(20,gyroangle);
                if (gyroangle>=18&&gyroangle<=22&&CurrentTime>=5){
                    state = State.Forward;
                    time.reset();
                    robot.Kill();
                }
                break;

            case center:
                robot.TurnAbsoulte(0,gyroangle);
                if (gyroangle>=-2&&gyroangle<=2&&CurrentTime>=5){
                    state = State.Forward;
                    time.reset();
                    robot.Kill();
                }
                break;

            case right:
                robot.TurnAbsoulte(-20,gyroangle);
                if (gyroangle>=-22&&gyroangle<=-18&&CurrentTime>=5){
                    state = State.Forward;
                    time.reset();
                    robot.Kill();
                }
                break;

            case Forward:
                robot.Forward(30,1);
                if(robot.DriveDone(30)){
                    state = State.Stop;
                    time.reset();
                    robot.Kill();
                }
                break;

            case Stop:
                time.reset();
                robot.Kill();
                break;

        }
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
