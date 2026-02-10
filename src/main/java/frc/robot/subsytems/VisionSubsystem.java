// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsytems;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.estimation.TargetModel;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.apriltag.*;

import java.io.IOException;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
  AprilTagFieldLayout tagLayout;
  VisionSystemSim visionSim;
  SwerveSubsystem m_swerveSubsystem;
  PhotonCamera camera;
  PhotonCameraSim cameraSim;
  public int targetID;

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

      this.tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
      visionSim.addAprilTags(tagLayout);

    } catch (IOException e) {
      Logger.recordOutput("VISION SIM ERROR", "TRUE");
    }

    // SETTING UP CAMERA IN SIMULATION
    camera = new PhotonCamera("mainCamera"); // placeholder camera for the simulation
    SimCameraProperties cameraProp = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProp); // the simulation using the camera and the properties

    Translation3d robotToCameraTrans = new Translation3d(.1, 0, .5); // camera location from robot pose
    Rotation3d robotToCameraRotation = new Rotation3d(0, 0/* Math.toRadians(-15) */, 0);
    Transform3d robotToCamera = new Transform3d(robotToCameraTrans, robotToCameraRotation);

    visionSim.addCamera(cameraSim, robotToCamera);

    cameraSim.enableRawStream(true);
    cameraSim.enableProcessedStream(true);
    cameraSim.enableDrawWireframe(true);

  }

  public int getBestTargetId() {
    var results = camera.getLatestResult(); // all the info about the detected targets

    boolean hasTargets = results.hasTargets(); // is there any valid targets??

    if (results.hasTargets()) {
      PhotonTrackedTarget target = results.getBestTarget(); // best target

      return target.getFiducialId();
    } else
      return -1;
  }

  @Override
  public void periodic() {
    var results = camera.getLatestResult(); // all the info about the detected targets

    boolean hasTargets = results.hasTargets(); // is there any valid targets??

    if (results.hasTargets()) {
      List<PhotonTrackedTarget> targets = results.getTargets(); // gets all of the targets
      PhotonTrackedTarget target = results.getBestTarget(); // best target
        }

    visionSim.update(m_swerveSubsystem.getPose2d());

    visionSim.getDebugField();
  }
}
