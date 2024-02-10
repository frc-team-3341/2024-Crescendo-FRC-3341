// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RIOVision extends SubsystemBase {
  Thread m_visionThread;

  /** Creates a new RIOVision. */
  public RIOVision() {
    m_visionThread = new Thread(
        () -> {

          int w = 100;
          int h = 100;

          
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture(0);       // Set the resolution


          camera.setPixelFormat(PixelFormat.kMJPEG);
          camera.setResolution(w, h);
          camera.setFPS(30);

          

          // Might work to save bandwidth
          // camera.setConnectionStrategy(ConnectionStrategy.kForceClose);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          // Works!!!

          // CvSource outputStream = CameraServer.putVideo("Rectangle", w, h);

          // Mats are very memory expensive. Lets reuse this Mat.
         // Mat mat = new Mat();

          // Mat original = new Mat();

          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
         /*  while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat. If there is an error notify the output.
            if (cvSink.grabFrameNoTimeout(original) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }

            // DO NOT TRANSLATE BETWEEN GREY AND RGB -> HURTS PERFORMANCE
            // CONSUMES MEMORY + PERFORMANCE TO SWITCH BETWEEN GREY AND RGB MAT
            Imgproc.cvtColor(original, mat, Imgproc.COLOR_BGR2GRAY);

            // Give the output stream a new image to display
           // outputStream.putFrame(mat);

          } */
        });
    m_visionThread.setDaemon(true);
    m_visionThread.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
