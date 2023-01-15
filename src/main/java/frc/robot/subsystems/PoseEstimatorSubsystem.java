// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.VISION.*;

public class PoseEstimatorSubsystem extends SubsystemBase {
  private Mecanum mecanum;
  private PhotonCamera limelight;
  private Gyro gyro;
  // private ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  private Field2d field2d = new Field2d();
  private double previousPipelineTimestamp = 0;

  // Unmodifed list 
  private static final List<Pose3d> targetPoses = Collections.unmodifiableList(
    List.of(
    new Pose3d(TAG1_X, TAG1_Y, TAG1_Z, new Rotation3d(0, 0, TAG1_THETA)),
    new Pose3d(TAG2_X, TAG2_Y, TAG2_Z, new Rotation3d(0, 0, TAG2_THETA))
    )
    );

  // State standard deviation of the pose estimate, indicated how much you trust your estimation. Increase more, less trust
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);
  // Standard deviation of the vision measurements
  private static final Vector<N3> visionMeas = VecBuilder.fill(0.05, 0.05, 0.05);

  private final MecanumDrivePoseEstimator poseEstimator;
  /** Creates a new PoseEstimatorSubsystem. */
  public PoseEstimatorSubsystem(Mecanum meca, PhotonCamera camera, Gyro gyro2) {
    this.limelight = camera;
    this.mecanum = meca;
    this.gyro = gyro2;

    poseEstimator = new MecanumDrivePoseEstimator(
      mecanum.getKinematics(), 
      new Rotation2d(Units.degreesToRadians(gyro.getYaw())), 
      new MecanumDriveWheelPositions(
        mecanum.getEncoder().get(0), 
        mecanum.getEncoder().get(1), 
        mecanum.getEncoder().get(2),
        mecanum.getEncoder().get(3)
        ), 
      initialPose,
      stateStdDevs,
      visionMeas
    );   
  }

  public Pose2d getPose2d() {
    return poseEstimator.getEstimatedPosition();
  } 

  @Override 
  public void periodic() {
    var pipelineResult = limelight.getLatestResult();
    var resultTimeStamp = pipelineResult.getTimestampSeconds();
    if(resultTimeStamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
      previousPipelineTimestamp = resultTimeStamp;
      var target = pipelineResult.getBestTarget();
      var fiducialId = target.getFiducialId();
      if(target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && fiducialId <= targetPoses.size()) {
        var targetPose = targetPoses.get(fiducialId);
        Transform3d camToTarget = target.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

        var visionMeasurement = camPose.transformBy(CAMERA_TO_ROBOT);
        poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimeStamp);
      }

    }
    poseEstimator.update(
      new Rotation2d(gyro.getYaw()), 
      new MecanumDriveWheelPositions(
        mecanum.getEncoder().get(0), 
        mecanum.getEncoder().get(1), 
        mecanum.getEncoder().get(2),
        mecanum.getEncoder().get(3)
        ) 
    );
    field2d.setRobotPose(getPose2d());
    // This method will be called once per scheduler run
  }
}
