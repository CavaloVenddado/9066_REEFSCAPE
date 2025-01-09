package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.apriltag.AprilTagFields;

public final class Constants {
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAngularSpeed = .5 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 4; // radians per second
    public static final double kMagnitudeSlewRate = 100; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    public static final double kTrackWidth = 0.59;
    public static final double kWheelBase = 0.59;
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 13;
    public static final int kRearLeftDrivingCanId = 11;
    public static final int kFrontRightDrivingCanId = 17;
    public static final int kRearRightDrivingCanId = 15;

    public static final int kFrontLeftTurningCanId = 12;
    public static final int kRearLeftTurningCanId = 10;
    public static final int kFrontRightTurningCanId = 16;
    public static final int kRearRightTurningCanId = 14;

    public static final boolean kGyroReversed = false;
    public static final int kPigeonCanId = 20;

    public static final double kSpeakerAimingP = 0.03;
    public static final double kSpeakerAimingI = 0;
    public static final double kSpeakerAimingD = 0.001;
    public static final double kSpeakerAimingTolerance = 1;

    public static final Translation3d shooterPivotPos = new Translation3d(0, 0.096, 0.35);

    public static final double kNoteAlignP = 0.01;
    public static final double kNoteAlignI = 0;
    public static final double kNoteAlignD = 0.001;
    public static final double kNoteAlignTolerance = 0.5;
  }

  public static final class ModuleConstants {
    public static final int kDrivingMotorPinionTeeth = 14;
    public static final boolean kTurningEncoderInverted = true;

    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }
 
  public static final class VisionConstants {
    public static final String kFrontCameraName = "D435_RGB";
    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final Transform3d kRobotToCam = new Transform3d(
      new Translation3d(0.20, 0.30, 0.655), 
      new Rotation3d(-0.262, 0, 0));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(5, 5, 20);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(1, 1, 20);

    public static final PoseStrategy kMultiTagStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    public static final PoseStrategy kSingleTagStrategy = PoseStrategy.LOWEST_AMBIGUITY;
    public static final String kBackCameraName = "REDRAGON_Live_Camera";
  }

  public static final class FieldConstants {
    public static final Pose3d speakerAimPoint_blue = new Pose3d(0, 5.55, 1.98 + 0.15, 
    new Rotation3d(0, 0, 0));

    public static final Pose3d speakerAimPoint_red = new Pose3d(16.58, 5.55, 1.98 + 0.15, 
    new Rotation3d(0, 0, 0));
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}