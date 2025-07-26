// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Utilities {
        public static Alliance getAlliance() {
                return DriverStation.getAlliance().get();
        }

        public static double getRadians(Pose2d pose) {
                return pose.getRotation().getRadians();
        }

        public static boolean inTolerance(double error, double tolerance) {
                return Math.abs(error) < tolerance;
        }

        public static boolean isValidPose(Pose2d pose) {
                return !pose.equals(new Pose2d()) && pose != null;
        }

        public static double getDistance(Pose2d current, Pose2d target) {
                return Math.hypot(target.getX() - current.getX(), target.getY() - current.getY());
        }

        public static int getClosestSide(Pose2d robotPose, Pose2d[] poses) {
                int side = 0;
                double leastDist = Double.MAX_VALUE;

                for (int i = 0; i < poses.length; i++) {
                        Pose2d pose = poses[i];
                        double distance = getDistance(robotPose, pose);

                        if (distance < leastDist) {
                                side = i;
                                leastDist = distance;
                        }
                }

                return side;
        }
}
