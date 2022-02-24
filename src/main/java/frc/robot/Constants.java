// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // dimensions
    private static final double ROBOT_SCALE = 1.0 / 5.0;
    public static final double TRACK_WIDTH = 0.141 / ROBOT_SCALE;

    // Spark Ports
    public static final int LEFT_MOTOR = 0;
    public static final int RIGHT_MOTOR = 1;

    // Encoder Ports
    public static final int LEFT_ENCODER_A = 4;
    public static final int LEFT_ENCODER_B = 5;
    public static final int RIGHT_ENCODER_A = 6;
    public static final int RIGHT_ENCODER_B = 7;

    // Encoders
    public static final double COUNTS_PER_REVOLUTION = 1440.0;
    public static final double WHEEL_DIAMETER_METER = 0.070 / ROBOT_SCALE;
    public static final double DISTANCE_PER_PULSE = (Math.PI * WHEEL_DIAMETER_METER) / COUNTS_PER_REVOLUTION;

    // Odometry
    public static final double START_X = 0.0;
    public static final double START_Y = 0.0;

    public static final Rotation2d START_ANG = new Rotation2d(0.0);

    public static final Pose2d START_POSE = new Pose2d(START_X, START_Y, START_ANG);
}
