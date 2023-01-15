// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.MOTOR_ID.*;
import static frc.robot.Constants.VISION.*;

import java.util.ArrayList;
import java.util.List;
public class Mecanum extends SubsystemBase {
  /** Creates a new Mecanum. */
  private WPI_TalonSRX leftFront = new WPI_TalonSRX(LEFT_FRONT);
  private WPI_TalonSRX rightFront = new WPI_TalonSRX(RIGHT_FRONT);
  private WPI_TalonSRX leftBack = new WPI_TalonSRX(LEFT_BACK);
  private WPI_TalonSRX rightBack = new WPI_TalonSRX(RIGHT_BACK);
  private MecanumDrive mecanum = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
  private List<Double> encoder = new ArrayList<>();
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(LEFT_FRONT_CENTER, RIGHT_FRONT_CENTER, LEFT_FRONT_CENTER, RIGHT_BACK_CENTER);
  // private MecanumDriveWheelPositions wheelPoses = new MecanumDriveWheelPositions();

  public Mecanum() {
    leftFront.setInverted(false);
    leftBack.setInverted(false);

    leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
  }

  public void drive (double x, double y, double zR) {
    mecanum.driveCartesian(x, y, zR);
  }

  public List<Double> getEncoder() {
    return encoder;
  }

  public MecanumDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoder.add(0, leftFront.getSelectedSensorPosition());
    encoder.add(1, rightFront.getSelectedSensorPosition());
    encoder.add(2, leftBack.getSelectedSensorPosition());
    encoder.add(3, rightBack.getSelectedSensorPosition());
  }
}
