// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private PhotonPipelineResult cameraResult;

  //StructPublisher<Pose2d> estPosePub = NetworkTableInstance.getDefault().getStructTopic("VisionEstimate", Pose2d.struct).publish();

  public VisionSubsystem() {

    camera = new PhotonCamera(VisionConstants.kFrontCameraName);

    // Setting up the photonEstimator to use the arena tag layout, the desired multi tag strategy, the front camera and the position of the camera relative to the robot
    photonEstimator = new PhotonPoseEstimator(
        VisionConstants.kTagLayout,
        VisionConstants.kMultiTagStrategy,
        VisionConstants.kRobotToCam);
    photonEstimator.setMultiTagFallbackStrategy(VisionConstants.kSingleTagStrategy); // Setting the strategy used in case only one tag is seen
  }

  // This function returns the robot pose with the timestamp and other resources. 
  // To use it as a Pose2D you need to write 
  // getEstimatedGlobalPose().get().estimatedPose.toPose2d()
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = photonEstimator.update(cameraResult);
    return visionEst;
  }
  
  // This function returns the standard deviations of the estimated pose, 
  // this is used to define how reliable that estimation actually is and 
  // determines when the swerve odometry is realigned by the estimation 
  // and how much it affects the position
  public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VisionConstants.kSingleTagStdDevs;
    var targets = camera.getLatestResult().getTargets();
    int numTags = 0;
    double avgDist = 0;

    for (var tgt : targets) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
      if (tagPose.isEmpty())
        continue;
      numTags++;
      avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0)
      return estStdDevs;
    avgDist /= numTags;
    
    // Decrease std devs if multiple targets are visible
    if (numTags > 1)
      estStdDevs = VisionConstants.kMultiTagStdDevs;
    
      // Increase std devs based on (average) distance
    if (numTags == 1 && avgDist > 6)
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    else
        estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

    return estStdDevs;
  }

  @Override
  public void periodic() {}
}