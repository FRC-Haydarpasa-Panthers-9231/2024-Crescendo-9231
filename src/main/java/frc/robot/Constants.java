// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
/*   "p": 0.4,
    "i": 0,
    "d": 0.01
 */  
  public static final double MaxModuleSpeed =5.3;
  public static final double driveBaseRadius = 0.51;

 public static class PIDConstants{
      public static final double kSwerveAutoPIDP = 15;
      public static final double kSwerveAutoPIDI = 0;
      public static final double kSwerveAutoPIDD = 0.1;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort=1;
    public static final int kPracticeControllerPort=2;
    public static final double LEFTX_DEADBAND = 0.1;
    public static final double LEFTY_DEADBAND = 0.1;
    public static final double RIGHTX_DEADBAND = 0.1;
  }

  public static class IntakeConstants {
    public static final int PIVOT_MOTOR_PORT = 9;
    public static final int ROLLER_MOTOR_PORT = 15;
    public static final double PIVOT_POWER = 0.3;
    public static final double AMP_SHOOT_POWER = 6;
    public static final double ROLLER_POWER = 11.5;
    public static final double INTAKE_P=0.008;
    public static final double INTAKE_I=0.00;
    public static final double INTAKE_D=0.0005;
    public static final int kCPR=8192;
  }

  public static class ShooterConstant {
    public static final int ROLLER_MOTOR1_PORT= 18;
    public static final int ROLLER_MOTOR2_PORT= 19;
    public static final int PIVOT_MOTOR1_PORT= 16;
    public static final int PIVOT_MOTOR2_PORT= 17;
    public static final double PIVOT_POWER=0.5;
    public static final double AMP_POWER=2.5; //11.5 speaker 2 amfi
    public static final double SPEAKER_POWER=11.5;
  }

  public static class ClimberConstant
  {
    public static final int CLIMBER1_PORT=20;
    public static final int CLIMBER2_PORT=21;
    public static final double CLIMBER_POWER=1;
  }
}
