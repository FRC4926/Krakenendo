// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  //PhotonCamera smallCamera = new PhotonCamera("smallcam");
  // PIDController test = new PIDController(1, 0, 0);


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // CameraServer.startAutomaticCapture();
    // // Creates the CvSink and connects it to the UsbCamera
   //CvSink cvSink = CameraServer.getVideo();

    // // Creates the CvSource and MjpegServer [2] and connects them
    // CvSource outputStream = CameraServer.putVideo("Cam", 640, 480);
  }

  @Override
  public void robotPeriodic() {
    // var smallResult = smallCamera.getLatestResult();
    // // var bigResult = bigCamera.getLatestResult();
    // SmartDashboard.putBoolean("Camera is connected", smallCamera.isConnected());
    // // //SmartDashboard.putNumber("Camera pipeline index", smallCamera.getPipelineIndex());
    // // //SmartDashboard.putBoolean("Camera has targets", result.hasTargets());
    // SmartDashboard.putNumber("April tag Small Index",
    //   smallResult.hasTargets() ? smallResult.getBestTarget().getFiducialId() : -1);


    // SmartDashboard.putNumber("April tag Big Index",
    //   bigResult.hasTargets() ? bigResult.getBestTarget().getFiducialId() : -1);

    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    // test.setSetpoint(0);
    // test.setTolerance(0.3);
        
    //   var smallResult = smallCamera.getLatestResult();
    //   if (smallResult.hasTargets())
    //   {
    //     double speed = test.calculate(smallResult.getBestTarget().getYaw());
    //     SmartDashboard.putNumber("PIDSpeed", speed);
    //     RobotContainer.drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(0) // Drive forward with negative Y (forward)
    //             .withVelocityY(speed) // Drive left with negative X (left)
    //             .withRotationalRate(0) // Drive counterclockwise with negative X (left)
    //         );
        
    //   }
  }

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
