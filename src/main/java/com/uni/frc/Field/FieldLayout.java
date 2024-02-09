package com.uni.frc.Field;


import java.util.HashMap;


import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;






public class FieldLayout {


    public static class Red {
        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();


        private static final Pose2d kTag1ToCenterAlign = new Pose2d(-3, 0.0, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag1ToRightAlign = new Pose2d(3, -0.55, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag1ToLeftAlign = new Pose2d(3, 0.55, Rotation2d.fromDegrees(0));


        private static final Pose2d kTag2ToCenterAlign = new Pose2d(0.77, 0.0, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag2ToRightAlign = new Pose2d(0.77, -0.55, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag2ToLeftAlign = new Pose2d(0.77, 0.55, Rotation2d.fromDegrees(0));


        private static final Pose2d kTag3ToCenterAlign = new Pose2d(-3, 0.0, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag3ToRightAlign = new Pose2d(-3, -0.55, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag3ToLeftAlign = new Pose2d(-3, 0.55, Rotation2d.fromDegrees(0));


        private static final Pose2d kTag4ToCenterAlign = new Pose2d(-3, 0.0, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag4ToRightAlign = new Pose2d(3, -0.55, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag4ToLeftAlign = new Pose2d(3, 0.55, Rotation2d.fromDegrees(0));




        private static final Pose2d kTag5ToCenterAlign = new Pose2d(0, -0.5, Rotation2d.fromDegrees(90));
        private static final Pose2d kTag5ToRightAlign = new Pose2d(0.77, -0.55, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag5ToLeftAlign = new Pose2d(0.77, 0.55, Rotation2d.fromDegrees(0));


        public static final AprilTag kAprilTag1 = new AprilTag(
                1,
                1.45,
                new Pose2d(
                        new Translation2d(15.94,0.7),
                        Rotation2d.fromDegrees(0)
                ),
                true,
                kTag1ToCenterAlign,
                kTag1ToCenterAlign,
                kTag1ToCenterAlign
        );


        public static final AprilTag kAprilTag2 = new AprilTag(
                2,
                1.45,
                new Pose2d(
                        new Translation2d(15.44,0.44),
                        Rotation2d.fromDegrees(0)
                ),
                true,
                kTag2ToCenterAlign,
                kTag2ToCenterAlign,
                kTag2ToCenterAlign
        );


        public static final AprilTag kAprilTag3 = new AprilTag(
                3,
                1.45,
                new Pose2d(
                        new Translation2d(16.5,5.25),
                        Rotation2d.fromDegrees(0)
                ),
                true,
                kTag3ToCenterAlign,
                kTag3ToCenterAlign,
                kTag3ToCenterAlign
        );


        public static final AprilTag kAprilTag4 = new AprilTag(
                4,
                1.45,
                new Pose2d(
                        new Translation2d(16.5,5.53),
                        Rotation2d.fromDegrees(0)
                ),
                true,
                kTag4ToCenterAlign,
                kTag4ToCenterAlign,
                kTag4ToCenterAlign
        );


        public static final AprilTag kAprilTag5 = new AprilTag(
                    5,
                    1.37,
                    new Pose2d(
                            new Translation2d(14.67,8.15),
                            Rotation2d.fromDegrees(0)
                    ),
                    true,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );


        static {
        kAprilTagMap.put(1, kAprilTag1);
        kAprilTagMap.put(2, kAprilTag2);
        kAprilTagMap.put(3, kAprilTag3);
        kAprilTagMap.put(4, kAprilTag4);
        kAprilTagMap.put(5, kAprilTag5);
        }
    }
}

