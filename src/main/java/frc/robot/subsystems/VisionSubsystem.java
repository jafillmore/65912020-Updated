/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.VisConst;
import frc.robot.TargetPipeline;

public class VisionSubsystem extends SubsystemBase {
  UsbCamera driveCam = CameraServer.getInstance().startAutomaticCapture();
  UsbCamera targetCam = CameraServer.getInstance().startAutomaticCapture();

 

  
  public int count = 0;
  public int pipeCount = 0;
  private VisionThread visionThread;
  private double centerX = 0.0;
  public double targetError = 0.0;
  
  private final Object imgLock = new Object();

public VisionSubsystem() {

  
  driveCam.setVideoMode(VideoMode.PixelFormat.kMJPEG,
                        VisConst.DriveCameraFrameWidth,
                        VisConst.DriveCameraFrameHeight,
                        VisConst.DriveCameraFPS);  
  
  

  
  targetCam.setVideoMode(VideoMode.PixelFormat.kMJPEG,
                        VisConst.TargetCameraFrameWidth,
                        VisConst.TargetCameraFrameHeight,
                        VisConst.TargetCameraFPS);

  targetCam.setBrightness(VisConst.TargetCameraBrightness);
  targetCam.setExposureAuto();
  //targetCam.setExposureManual(VisConstants.targetCameraExposure);

  CvSource outputStream = CameraServer.getInstance().putVideo("Processed in Main", VisConst.TargetCameraFrameWidth, VisConst.TargetCameraFrameHeight);
  
  visionThread = new VisionThread(targetCam, new TargetPipeline(), targetPipeline -> {
                          
                 
      SmartDashboard.putNumber("Contours Found", targetPipeline.findContoursOutput().size());
                                     
      if (!targetPipeline.filterContoursOutput().isEmpty()) {
        
        SmartDashboard.putNumber("Filtered Contours", targetPipeline.filterContoursOutput().size());
  
          Rect r = Imgproc.boundingRect(targetPipeline.filterContoursOutput().get(0));
          synchronized (imgLock) {
              centerX = r.x + (r.width / 2);
  
              SmartDashboard.putNumber("Center X from Subsys VisionThread", centerX);
  
              targetError = centerX - (VisConst.TargetCameraFrameWidth/2);
  
            
          }
          outputStream.putFrame(targetPipeline.cvAbsdiffOutput);
      }
    
    });
    
    visionThread.start();   


}
 

@Override
public void periodic() {
  // This method will be called once per scheduler run

  synchronized (imgLock) {
   SmartDashboard.putNumber("Center X from Subsys VisionThread", centerX);
    

  }

}
}
