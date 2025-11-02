// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import java.io.IOException;

import org.ejml.dense.block.MatrixOps_DDRB;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  AprilTagFieldLayout tagLayout;
  VisionSystemSim visionSim;
  SwerveSubsystem m_swerveSubsystem;
  
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;

    // SETTING UP SIMULATION
    this.visionSim = new VisionSystemSim("main");
    TargetModel targetModel = new TargetModel(0.5, 0.25);
   
    Pose3d targetPose = new Pose3d(16, 4, 2, new Rotation3d(0, 0, Math.PI));
    VisionTargetSim visionTarget = new VisionTargetSim(targetPose, targetModel);
    
    visionSim.addVisionTargets(visionTarget);
    
    try {
      this.tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (IOException e) {
      Logger.recordOutput("VISION SIM ERROR", "TRUE");
    }

    visionSim.addAprilTags(tagLayout);

    // SETTING UP CAMERA IN SIMULATION
    SimCameraProperties cameraProp = new SimCameraProperties();

    PhotonCamera camera = new PhotonCamera("mainCamera");

    PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);

    Translation3d robotToCameraTrans = new Translation3d(.1, 0, .5); // camera location from robot pose

    Rotation3d robotToCameraRotation = new Rotation3d(0, Math.toRadians(-15), 0);

    Transform3d robotToCamera = new Transform3d(robotToCameraTrans, robotToCameraRotation);
    
    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);
    
  }
  
  @Override
  public void periodic() {
    visionSim.update(m_swerveSubsystem.getPose2d());
    
    visionSim.getDebugField();
  }
}
