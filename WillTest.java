package org.firstinspires.ftc.teamcode.ArkhamAuto;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ArkhamTele.ArkhamHW;
import org.firstinspires.ftc.teamcode.ArkhamTele.ArkhamSensors;

import java.util.Locale;




@Autonomous(name = "ArkhamSample")


public class ArkhamSample extends OpMode {

    ArkhamHW robot = new ArkhamHW();
    ArkhamSensors sensors = new ArkhamSensors();
    public SamplingOrderDetector detector;

    enum State {
        Start,left,center,right, Sample, Stop, LForward, RForward, CForward, LTurn1,
        LTurn2, LDriveForward1, LDriveForward2, RTurn1, RDriveForward1, RTurn2, RDriveForward2,
        RTurn3, RDriveForward3, CTurn1, CDriveForward1, CTurn2, CDriveForward2, CTurn3,
        CDriveForward3}

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
                robot.TurnAbsolute(25,gyroangle);
                if (gyroangle>=23&&gyroangle<=27&&CurrentTime>=2){
                    state = State.LForward;
                    robot.Kill();
                    time.reset();
                }
                break;

            case LForward:
                robot.Forward(1,49);
                if(robot.DriveDone(49)){
                    robot.Kill();
                    state = state.LTurn1;
                    time.reset();
                }
                break;

            case LTurn1:
                robot.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RF.setPower(.3);
                if (gyroangle<=175) {
                    robot.Kill();
                    state = state.LDriveForward1;
                    time.reset();
                }
                break;

            case LDriveForward1:
                robot.Forward(1,17);
                if (robot.DriveDone(17)){
                    robot.Kill();
                    state = State.LTurn2;
                    time.reset();

                }
                break;

            case LTurn2:
                robot.TurnAbsolute ( -135, gyroangle);
                if (gyroangle>=-132&&gyroangle<=-137) {
                    robot.Kill();
                    state = state.LDriveForward2;
                    time.reset();
                }
                break;

            case LDriveForward2:
                robot.Forward(1,60);
                if (robot.DriveDone(60)){
                    robot.Kill();
                    state = State.Stop;
                    time.reset();

                }
                break;


            case center:
                robot.TurnAbsolute(0,gyroangle);
                if (gyroangle>=-2&&gyroangle<=2){
                    state = State.CForward;
                    time.reset();
                    robot.Kill();
                }
                break;



            case CForward:
                robot.Forward(1,49);
                if(robot.DriveDone(49)){
                    robot.Kill();
                    state = state.CTurn1;
                    time.reset();
                }
                break;

            case CTurn1:
                robot.TurnAbsolute ( -90, gyroangle);
                if (gyroangle>=-88&&gyroangle<=-92) {
                    robot.Kill();
                    state = state.CDriveForward1;
                    time.reset();
                }
                break;

            case CDriveForward1:
                robot.Forward(1,17);
                if(robot.DriveDone(17)){
                    robot.Kill();
                    state = state.CTurn2;
                    time.reset();
                }
                break;

            case CTurn2:
                robot.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RF.setPower(.3);
                if (gyroangle<=175) {
                    robot.Kill();
                    state = state.CDriveForward2;
                    time.reset();
                }
                break;

            case CDriveForward2:
                robot.Forward(1,50);
                if(robot.DriveDone(50)){
                    robot.Kill();
                    state = state.CTurn3;
                    time.reset();
                }
                break;

            case CTurn3:
                robot.TurnAbsolute ( -135, gyroangle);
                if (gyroangle>=-133&&gyroangle<=-137) {
                    robot.Kill();
                    state = state.CDriveForward3;
                    time.reset();
                }
                break;

            case CDriveForward3:
                robot.Forward(1,36);
                if(robot.DriveDone(36)){
                    robot.Kill();
                    state = state.Stop;
                    time.reset();
                }
                break;


            case right:
                robot.TurnAbsolute(-25,gyroangle);
                if (gyroangle>=-27&&gyroangle<=-23){
                    robot.Kill();
                    state = State.RForward;
                    time.reset();
                }
                break;

                case RForward:
                robot.Forward(1,49);
                if(robot.DriveDone(49)){
                    robot.Kill();
                    state = state.RTurn1;
                    time.reset();
                }
                break;

            case RTurn1:
                robot.TurnAbsolute(-90,gyroangle);
                if (gyroangle>=-88&&gyroangle<=92){
                    robot.Kill();
                    state = State.RDriveForward1;
                    time.reset();
                }
                break;

            case RDriveForward1:
                robot.Forward(1,17);
                if(robot.DriveDone(17)){
                    robot.Kill();
                    state = state.RTurn2;
                    time.reset();
                }
                break;

            case RTurn2:
                robot.RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.RF.setPower(.3);
                if (gyroangle<=175){
                    robot.Kill();
                    state = State.RDriveForward2;
                    time.reset();
                }
                break;

            case RDriveForward2:
                robot.Forward(1,67);
                if(robot.DriveDone(67)){
                    robot.Kill();
                    state = state.RTurn3;
                    time.reset();
                }
                break;

            case RTurn3:
                robot.TurnAbsolute(-135,gyroangle);
                if (gyroangle>=-133&&gyroangle<=-137){
                    robot.Kill();
                    state = State.RDriveForward3;
                    time.reset();
                }
                break;

            case RDriveForward3:
                robot.Forward(1,36);
                if(robot.DriveDone(36)){
                    robot.Kill();
                    state = state.Stop;
                    time.reset();
                }
                break;

            case Stop:
                time.reset();
                robot.Kill();
                detector.disable();
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
