// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     *  The left to right distance between the drivetrain wheels
     * 
     *  should be measured from the center to center
     */
    public static final double Drivetrain_Trackwidth_meters = Units.inchesToMeters(26.0);
    /**
     *  The front to back distance between the drivetrain wheels
     * 
     *  should be measured from the center to center
     */
    public static final double Drivetrain_Wheelbase_meters = Units.inchesToMeters(26.1);
    public static final int drivetrainPideonID = 0;
    // Front left module 
    public static final int FrontLeftModuleDriveMotor = 1;
    public static final int FrontLeftModuleSteerEncoder = 2;
    public static final int FrontLeftModuleSteerMotor = 3;
    public static final double FrontLeftModuleSteerOffset = -Math.toRadians(0.0);
    // front right module
    public static final int FrontRightModuleDriveMotor = 4;
    public static final int FrontRightModuleSteerEncoder = 5;
    public static final int FrontRightModuleSteerMotor = 6;
    public static final double FrontRightModuleSteerOffset = -Math.toRadians(0.0);
    // Back left module
    public static final int BackLeftModuleDriveMotor = 7;
    public static final int BackLeftModuleSteerEncoder = 8;
    public static final int BackLeftModuleSteerMotor = 9;
    public static final double BackLeftModuleSteerOffset = -Math.toRadians(0.0);
    // Back right module
    public static final int BackRightModuleDriveMotor = 10;
    public static final int BackRightModuleSteerEncoder = 11;
    public static final int BackRightModuleSteerMotor = 12;
    public static final double BackRightModuleSteerOffset = -Math.toRadians(0.0);
    


}

