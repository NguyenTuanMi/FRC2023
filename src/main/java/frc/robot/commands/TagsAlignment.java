// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Mecanum;

import static frc.robot.Constants.VISION.*;
public class TagsAlignment extends CommandBase {
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
  private static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

  private static final int TAG_TO_CHASE = 2;
  private static final Transform3d TAG_TO_GOAL = 
  new Transform3d(
    new Translation3d(1.5, 0, 0), 
    new Rotation3d(0, 0, Math.PI)
  );

  private PhotonCamera camera;
  private Mecanum mecanum;
  private Supplier<Pose2d> poseProvider;

  private ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, X_CONSTRAINTS);
  private ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, Y_CONSTRAINTS);
  private ProfiledPIDController rController = new ProfiledPIDController(0, 0, 0, THETA_CONSTRAINTS);

  private PhotonTrackedTarget lastTarget;
  /** Creates a new TagsAlignment. */
  public TagsAlignment(PhotonCamera camera, Mecanum drivetrain, Supplier<Pose2d> supplier ) {
    this.camera = camera;
    this.mecanum = drivetrain;
    this.poseProvider = supplier;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    rController.setTolerance(0.2);
    rController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = poseProvider.get();
    rController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var robotPose2d = poseProvider.get();
    var robotPose = new Pose3d(
      robotPose2d.getX(),
      robotPose2d.getY(),
      0,
      new Rotation3d(0, 0, robotPose2d.getRotation().getRadians())
    );

    var photonRes = camera.getLatestResult();
    if(photonRes.hasTargets()) {
      var targetOpt = photonRes.getTargets().stream()
      .filter(t -> t.getFiducialId() == TAG_TO_CHASE) 
      .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2)
      .findFirst();
      if(targetOpt.isPresent()) {
        var target = targetOpt.get();
        lastTarget = target;
        
        var cameraPose = robotPose.transformBy(CAMERA_TO_ROBOT.inverse());

        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        rController.setGoal(goalPose.getRotation().getRadians());

      }
    }

    if(lastTarget == null) {
      mecanum.drive(0, 0, 0);
    }
    else {
      var xSpeed = xController.calculate(robotPose.getX());
      if(xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose.getY());
      if(yController.atGoal()) {
        ySpeed = 0;
      }

      var rSpeed = rController.calculate(robotPose2d.getRotation().getRadians());
      if(rController.atGoal()) {
        rSpeed = 0;
      }
      mecanum.drive(xSpeed, ySpeed, rSpeed);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
