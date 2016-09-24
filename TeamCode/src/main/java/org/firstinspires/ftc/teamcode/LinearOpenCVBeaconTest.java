package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

/**
 * Created by jb on 9/24/16.
 */

public class LinearOpenCVBeaconTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber;

        frameGrabber.grabSingleFrame();
        while (!frameGrabber.isResultReady()){
            Thread.sleep(5);
        }

        ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
        BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();

        BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
        BeaconColorResult.BeaconColor rightColor = result.getRightColor();

        telemetry.addData("Result", result);
        telemetry.update();

        Thread.sleep(1000); //quiting clears telemetry data
    }

}
