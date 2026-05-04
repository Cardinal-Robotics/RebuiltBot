// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Robot;

// Our own wrapper class that can hold a PhotonCamera and all the other things we need to track with it for convinience.
public class Camera {
    private AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(
            AprilTagFields.k2026RebuiltWelded);
    private final PhotonPoseEstimator poseEstimator;
    private final Transform3d robotToCameraOffset;
    private final PhotonCamera photonCamera;
    private static VisionSystemSim visionSim;

    private Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();
    private List<PhotonPipelineResult> resultsList = new ArrayList<>();
    private PhotonCameraSim cameraSimulation;

    // Standard deviation settings for recognizing AprilTags?
    private final Matrix<N3, N1> singleTagStdDevs;
    private final Matrix<N3, N1> multiTagStdDevs;
    public Matrix<N3, N1> curStdDevs;

    public Camera(String cameraName, Transform3d robotToCamera, Matrix<N3, N1> singleTagStdDevs,
            Matrix<N3, N1> multiTagStdDevsMatrix) {
        this.photonCamera = new PhotonCamera(cameraName);
        this.robotToCameraOffset = robotToCamera;

        this.singleTagStdDevs = singleTagStdDevs;
        this.multiTagStdDevs = multiTagStdDevsMatrix;

        try {
            File tagLayoutJSON = new File(Filesystem.getDeployDirectory(), "practicefield.json");
            this.tagLayout = new AprilTagFieldLayout(tagLayoutJSON.toPath());
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        this.poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCameraOffset);
        this.poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        if (Robot.isSimulation())
            createSimulation();
    }

    public void createSimulation() {
        if(visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(tagLayout);
        }
        
        SimCameraProperties cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(100));
        cameraProperties.setCalibError(0.25, 0.08);
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        cameraSimulation = new PhotonCameraSim(photonCamera, cameraProperties);
        cameraSimulation.enableRawStream(true);
        cameraSimulation.enableProcessedStream(true);
        cameraSimulation.enableDrawWireframe(true);

        this.visionSim.addCamera(cameraSimulation, robotToCameraOffset);
    }

    /* ------ GETTERS LOGIC ------ */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return this.estimatedRobotPose;
    }

    public Optional<PhotonPipelineResult> getBestResult() {
        if (resultsList.isEmpty())
            return Optional.empty();

        PhotonPipelineResult bestResult = resultsList.get(0);
        double amiguity = bestResult.getBestTarget().getPoseAmbiguity();

        for (PhotonPipelineResult result : resultsList) {
            double currentAmbiguity = result.getBestTarget().getPoseAmbiguity();
            if (currentAmbiguity < amiguity && currentAmbiguity > 0) {
                bestResult = result;
                amiguity = currentAmbiguity;
            }
        }
        return Optional.of(bestResult);
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
        if (resultsList.isEmpty())
            return Optional.empty();
        return Optional.of(resultsList.get(0));
    }

    /* ------ VISION PROCESSING LOGIC ------ */

    public void update() {
        this.updateUnreadVisionResults();
        // If we've seen AprilTags recently, run pose estimation.
        if (!resultsList.isEmpty())
            updatePoseEstimation();
    }

    public static void updateSimulationSwerve(Pose2d pose) {
        visionSim.update(pose);
    }

    private void updateUnreadVisionResults() {
        double mostRecentTimestamp = resultsList.isEmpty() ? 0.0 : resultsList.get(0).getTimestampSeconds();

        for (PhotonPipelineResult result : resultsList) {
            mostRecentTimestamp = Math.max(mostRecentTimestamp, result.getTimestampSeconds());
        }

        // TODO: Check if this logic get simulation camera logic is necessary...
        resultsList = Robot.isReal() ? photonCamera.getAllUnreadResults()
                : cameraSimulation.getCamera().getAllUnreadResults();

        resultsList.sort((PhotonPipelineResult a, PhotonPipelineResult b) -> {
            return a.getTimestampSeconds() >= b.getTimestampSeconds() ? 1 : -1;
        });
    }

    private void updatePoseEstimation() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : resultsList) {
            visionEst = poseEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }

        this.estimatedRobotPose = visionEst;
    }

    /*
     * This was made by people smarter than me. It changes AprilTag recognition
     * settings or something in different contexts.
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose,
            List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // If pose estimation isn't working, default to single-tag std devs
            curStdDevs = singleTagStdDevs;
            return;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = singleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an
        // average-distance metric
        for (var target : targets) {
            var tagPose = poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty())
                continue;

            numTags++;
            avgDist += tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            // No tags visible. Default to single-tag std devs
            curStdDevs = singleTagStdDevs;
            return;
        }

        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;

        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = multiTagStdDevs;
        }
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        curStdDevs = estStdDevs;
    }
}
