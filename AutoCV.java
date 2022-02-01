/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoCV", group="Pushbot")
public class AutoCV extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMap21           robot   = new HardwareMap21();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = 229.1831181;
    // COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * π)
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    // adjustable values for easy testing
    static final double     ONE_TILE                = 12;
    static final double     NINETY_DEGREE_TURN      = 12;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.leftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // send telemetry message to show current wheels position
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.rightDrive.getCurrentPosition(),
                robot.leftDrive.getCurrentPosition());
        telemetry.update();




        // set up and initialize cameras
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // TODO: line below: might have to change for an external webCamera
        webCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new FreightFrenzyDeterminationPipeline();
        webCamera.setPipeline(pipeline);
        webCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webCamera.openCameraDeviceAsync(() -> {
          webCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        });
        OpenCvCamera webCamera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // wait for the game to start (driver presses PLAY)
        waitForStart();

        webCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
            }
            @Override
            public void onError(int errorCode)
            {
               /*
               * This will be called if the camera could not be opened
               */
            }
        });

        //stop
        robot.rightDrive.setPower(0);
        robot.leftDrive.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public static class FreightFrenzyDeterminationPipeline extends OpenCvPipeline
    {
      public enum Barcode
      {
        ONE,
        TWO,
        THREE
      }

      static final Scalar RED = new Scalar(255, 0, 0)
      static final Scalar GREEN = new Scalar(0, 255, 0)
      static final Scalar BLUE = new Scalar(0, 0, 255)

      // making the boxes to scan ducks
      // numbers subject to change
      static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point (91, 98);
      static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point (181, 98);
      static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point (271, 98);


      static final int REGION_WIDTH = 35;
      static final int REGION_HEIGHT = 25;

      final int ONE_THRESHOLD = 115;
      final int TWO_THRESHOLD = 135;
      final int THREE_THRESHOLD = 155;

      Point region1_pointA = new Point(
        REGION1_TOPLEFT_ANCHOR_POINT.x,
        REGION1_TOPLEFT_ANCHOR_POINT.y);
      Point region1_pointB = new Point(
        REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
        REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

      Mat region1_Cb;
      Mat YCrCb = new Mat();
      Mat CB = new Mat();
      int avg1;

      private volatile DuckPosition position = DuckPosition.TWO;

      void inputToCb(Mat input)
      {
        Imgproc.cutColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
      }

      @Override
      public void init(MatfirstFrame)
      {
        inputToCb(firstFrame);
        region1_Cb = Cb.submat(new Rect(region1_pointA, region_pointB));
      }

      @Override
      public Mat processFrame(Mat input)
      {
        inputToCb(input);
        avg1 = (int) Core.mean(region1_Cb).val[0];
        Imgproc.rectangle(
          input,
          region1_pointA,
          region1_pointB,
          RED,
          3);

        position = DuckPosition.TWO;
        if (avg1 > THREE_THRESHOLD) {
          position = DuckPosition.THREE;
        } else if (avg1 > TWO_THRESHOLD){
          position = DuckPosition.TWO;
        } else if (avg1 > ONE_THRESHOLD){
          position = DuckPosition.ONE;
        }

        Imgproc.rectangle(
          input,
          region1_pointA,
          region1_pointB,
          BLUE,
          -1);

        return input;

      }

      public int getAnalysis()
      {
        return avg1;
      }

    }


/* pipeline for greyscale conversion


    class ConvertToGreyPipeline extends OpenCvPipeline {
    // Notice this is declared as an instance variable (and re-used), not a local variable
    Mat grey = new Mat();

    @Override
    public Mat processFrame(Mat input)
      {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        return grey;
      }
    }
*/


/* example of how to create a pipeline, already done with processFrame


    class FoobarPipeline extends OpenCvPipeline {
    int lastResult = 0;

    @Override
    public Mat processFrame(Mat input)
    {
        // ... some image processing here ...

          if(...)
          {
              lastResult = ONE;
          }
          else if(...)
          {
              lastResult = TWO
          }
          else if(...)
          {
              lastResult = THREE;
          }
      }

      public int getLatestResults()
      {
          return lastResult;
      }
    }
*/





    // define function encoderDrive
    public void encoderDrive(double speed, double leftInches,double rightInches, double timeoutS) {

        // define variables
        int rightTarget;
        int leftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            rightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            robot.leftDrive.setTargetPosition(leftTarget);
            robot.rightDrive.setTargetPosition(rightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.rightDrive.isBusy() && robot.leftDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d",
                        leftTarget,
                        rightTarget
                );
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Reset encoders again
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
