// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.apriltag.*;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonPipelineResult;

import static edu.wpi.first.units.Units.Meters;

import java.io.File;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
  private AprilTagFieldLayout tagLayout;

  private final SwerveSubsystem m_swerveSubsystem;
  private final Camera leftCamera, rightCamera;

  public VisionSubsystem(SwerveSubsystem swerveSubsystem) {
    this.m_swerveSubsystem = swerveSubsystem;

    // try {
    //   File tagLayoutJSON = new File(Filesystem.getDeployDirectory(), "practicefield.json");
    //   this.tagLayout = new AprilTagFieldLayout(tagLayoutJSON.toPath());
    // } catch (Exception e) {
    //   throw new RuntimeException(e);
    // }

    // CHECK Camera.java
    this.tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);



    leftCamera = new Camera("Left",
        new Transform3d(
            new Translation3d(
              Meters.fromBaseUnits(0.29613),
              Meters.fromBaseUnits(-0.22033),
              Meters.fromBaseUnits(0.15914)
            ),
            new Rotation3d(Math.toRadians(180), Math.toRadians(66.602095), 0)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1));

    rightCamera = new Camera("Right",
        new Transform3d(
            new Translation3d(
              Meters.fromBaseUnits(0.29613),
              Meters.fromBaseUnits(-0.22033),
              Meters.fromBaseUnits(0.15914)
            ),
            new Rotation3d(0, Math.toRadians(45.0), 0)),
        VecBuilder.fill(4, 4, 8),
        VecBuilder.fill(0.5, 0.5, 1));
  }

  @Override
  public void periodic() {
    if(Robot.isSimulation()) Camera.updateSimulationSwerve(m_swerveSubsystem.getSwerveDrive().getSimulationDriveTrainPose().get());

    // Processes the latest vision data and updates the camera's pose estimation.
    //this.leftCamera.update();
    this.rightCamera.update();

    // If we have a pose estimate from a camera, give it to YAGSL to do vision +
    // wheel movement odometry.
    Optional<EstimatedRobotPose> rightPoseEstimate = rightCamera.getEstimatedPose();
    Optional<EstimatedRobotPose> leftPoseEstimate = leftCamera.getEstimatedPose();
    Logger.recordOutput("vision/rightEstimate", rightPoseEstimate.get().estimatedPose);
    Logger.recordOutput("vision/leftEstimate", leftPoseEstimate.get().estimatedPose);

    this.consumePoseEstimate(rightPoseEstimate, rightCamera.curStdDevs);
    this.consumePoseEstimate(leftPoseEstimate, leftCamera.curStdDevs);
  }

  public void consumePoseEstimate(Optional<EstimatedRobotPose> poseEstimate, Matrix<N3, N1> stdDevs) {
    if (poseEstimate.isEmpty())
      return;

    EstimatedRobotPose pose = poseEstimate.get();
    this.m_swerveSubsystem.addVisionMeasurement(pose, stdDevs);
  }

  public Optional<PhotonPipelineResult> getBestResult() {
    Optional<PhotonPipelineResult> leftCameraBest = this.leftCamera.getBestResult();
    Optional<PhotonPipelineResult> rightCameraBest = this.rightCamera.getBestResult();

    // If one is empty, return the other. If both are empty, return
    // Optional.empty();
    if (leftCameraBest.isPresent() && rightCameraBest.isEmpty())
      return leftCameraBest;
    else if (rightCameraBest.isPresent() && leftCameraBest.isEmpty())
      return rightCameraBest;
    else if (leftCameraBest.isEmpty() && rightCameraBest.isEmpty())
      return Optional.empty();

    // If both are present, return the one with the least ambiguity.
    double leftBestAmbiguity = leftCameraBest.get().getBestTarget().poseAmbiguity;
    double rightBestAmbiguity = rightCameraBest.get().getBestTarget().poseAmbiguity;

    if (leftBestAmbiguity > rightBestAmbiguity)
      return rightCameraBest;
    else
      return leftCameraBest;
  }
}
