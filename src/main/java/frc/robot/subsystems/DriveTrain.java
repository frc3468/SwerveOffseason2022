// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import static frc.robot.Constants.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  public static final Mk4iSwerveModuleHelper.GearRatio motorRatio = Mk4iSwerveModuleHelper.GearRatio.L1;

  public static final double maxVoltage = 12.0;

  public static final double maxVelocityMetersPerSecond = 5880.0/60.0 *
  SdsModuleConfigurations.MK4I_L1.getDriveReduction()*
  SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;

  public static final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
  Math.hypot(Drivetrain_Trackwidth_meters/2.0,Drivetrain_Wheelbase_meters / 2.0 );


  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // front left
    new Translation2d(Drivetrain_Trackwidth_meters / 2.0, Drivetrain_Wheelbase_meters / 2),
    // front right
    new Translation2d(Drivetrain_Trackwidth_meters / 2.0, -Drivetrain_Wheelbase_meters / 2),
    // back left
    new Translation2d(-Drivetrain_Trackwidth_meters / 2.0, Drivetrain_Wheelbase_meters / 2),
    // back right
    new Translation2d(-Drivetrain_Trackwidth_meters / 2.0, -Drivetrain_Wheelbase_meters / 2)
  );

  private final PigeonIMU m_pigeon = new PigeonIMU(drivetrainPideonID);

  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    //front left
    m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Front Left Module", BuiltInLayouts.kList)
    .withSize(2, 4)
    .withPosition(0, 0),
    Mk4iSwerveModuleHelper.GearRatio.L1,
     FrontLeftModuleDriveMotor,
     FrontLeftModuleSteerMotor,
     FrontLeftModuleSteerEncoder,
     FrontLeftModuleSteerOffset
     );

     //front right
     m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Front Right Module", BuiltInLayouts.kList)
    .withSize(2, 4)
    .withPosition(2, 0),
    Mk4iSwerveModuleHelper.GearRatio.L1,
     FrontRightModuleDriveMotor,
     FrontRightModuleSteerMotor,
     FrontRightModuleSteerEncoder,
     FrontRightModuleSteerOffset
     );

     //back left
     m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Back Left Module", BuiltInLayouts.kList)
    .withSize(2, 4)
    .withPosition(4, 0),
    Mk4iSwerveModuleHelper.GearRatio.L1,
     BackLeftModuleDriveMotor,
     BackLeftModuleSteerMotor,
     BackLeftModuleSteerEncoder,
     BackLeftModuleSteerOffset
     );

     //back right
     m_backRightModule = Mk4iSwerveModuleHelper.createNeo(tab.getLayout("Back Right Module", BuiltInLayouts.kList)
    .withSize(2, 4)
    .withPosition(6, 0),
    Mk4iSwerveModuleHelper.GearRatio.L1,
     BackRightModuleDriveMotor,
     BackRightModuleSteerMotor,
     BackRightModuleSteerEncoder,
     BackRightModuleSteerOffset
     );

  }

  public void zeroGyroscope() {
    m_pigeon.setFusedHeading(0.0);
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states,maxVelocityMetersPerSecond);

    m_frontLeftModule.set(states[0].speedMetersPerSecond/maxVelocityMetersPerSecond * maxVoltage, states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond/maxVelocityMetersPerSecond*maxVoltage, states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond/maxVelocityMetersPerSecond*maxVoltage, states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond/maxVelocityMetersPerSecond*maxVoltage, states[3].angle.getRadians());
  }
}