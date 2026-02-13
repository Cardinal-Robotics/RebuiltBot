// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.apriltag.*;

import frc.robot.subsystems.SwerveSubsystem;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
  private final Camera leftCamera, rightCamera;

  private SwerveSubsystem m_swerveSubsystem;
  private VisionSystemSim visionSim;

  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;

    leftCamera = new Camera("leftCamera", new Transform3d(
        new Translation3d(),
        new Rotation3d()),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1));

    rightCamera = new Camera("rightCamera", new Transform3d(
        new Translation3d(),
        new Rotation3d()),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1));
  }

  @Override
  public void periodic() {
    visionSim.update(m_swerveSubsystem.getPose2d());
    visionSim.getDebugField();

    // Processes the latest vision data and updates the camera's pose estimation.
    this.leftCamera.update();
    this.rightCamera.update();

    // If we have a pose estimate from a camera, give it to YAGSL to do vision +
    // wheel movement odometry.
    Optional<EstimatedRobotPose> rightPoseEstimate = rightCamera.getEstimatedPose();
    Optional<EstimatedRobotPose> leftPoseEstimate = leftCamera.getEstimatedPose();
    this.consumePoseEstimate(rightPoseEstimate);
    this.consumePoseEstimate(leftPoseEstimate);
  }

  public void consumePoseEstimate(Optional<EstimatedRobotPose> poseEstimate) {
    if (poseEstimate.isEmpty())
      return;

    EstimatedRobotPose pose = poseEstimate.get();
    this.m_swerveSubsystem.addVisionMeasurement(pose, leftCamera.curStdDevs);
  }

  public Optional<PhotonPipelineResult> getBestResult() {
    Optional<PhotonPipelineResult> leftCameraBest = this.leftCamera.getBestResult();
    Optional<PhotonPipelineResult> rightCameraBest = this.rightCamera.getBestResult();
    Optional<PhotonPipelineResult> bestResult;

    // If one is empty, return the other. If both are empty, return Optional.empty();  
    if(leftCameraBest.isPresent() && rightCameraBest.isEmpty()) return bestResult = leftCameraBest;
    else if(rightCameraBest.isPresent() && leftCameraBest.isEmpty()) return bestResult = rightCameraBest;
    else if(leftCameraBest.isEmpty() && rightCameraBest.isEmpty()) return bestResult = Optional.empty();

    double leftBestAmbiguity = leftCameraBest.get().getBestTarget().poseAmbiguity;
    double rightBestAmbiguity = rightCameraBest.get().getBestTarget().poseAmbiguity;

    if(leftBestAmbiguity > rightBestAmbiguity) return rightCameraBest;
    else return leftCameraBest;
  }
}
