package com.wcp.frc.Field;

import java.util.HashMap;

import com.wcp.lib.geometry.Pose2d;
import com.wcp.lib.geometry.Rotation2d;
import com.wcp.lib.geometry.Translation2d;



public class FieldLayout {

    public static class Red {
        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();
        // From field manual: 46.355 cm
        // From CAD: 46.2534 cm
        // From step layout marking: 46.2788 cm
        // Old value: 46.272  cm
        private static final Pose2d kTag1ToCenterAlign = new Pose2d(-3, 0.0, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag1ToRightAlign = new Pose2d(3, -0.55, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag1ToLeftAlign = new Pose2d(3, 0.55, Rotation2d.fromDegrees(0));

        private static final Pose2d kTag2ToCenterAlign = new Pose2d(0.77, 0.0, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag2ToRightAlign = new Pose2d(0.77, -0.55, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag2ToLeftAlign = new Pose2d(0.77, 0.55, Rotation2d.fromDegrees(0));

        private static final Pose2d kTag3ToCenterAlign = new Pose2d(0.77, 0.0, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag3ToRightAlign = new Pose2d(0.77, -0.55, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag3ToLeftAlign = new Pose2d(0.77, 0.55, Rotation2d.fromDegrees(0));

        private static final Pose2d kTag5ToCenterAlign = new Pose2d(0, -0.5, Rotation2d.fromDegrees(90));
        private static final Pose2d kTag5ToRightAlign = new Pose2d(0.77, -0.55, Rotation2d.fromDegrees(0));
        private static final Pose2d kTag5ToLeftAlign = new Pose2d(0.77, 0.55, Rotation2d.fromDegrees(0));
        public static final AprilTag kAprilTag4 = new AprilTag(
                4,
                1.355852,
                new Pose2d(
                        new Translation2d(16.7,5.53),
                        Rotation2d.fromDegrees(0)
                ),
                true,
                kTag1ToCenterAlign,
                kTag1ToCenterAlign,
                kTag1ToCenterAlign
        );
        public static final AprilTag kAprilTag5 = new AprilTag(
                    5,
                    1.355852,
                    new Pose2d(
                            new Translation2d(14.67,8.12),
                            Rotation2d.fromDegrees(0)
                    ),
                    true,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign,
                    kTag5ToCenterAlign
            );


        static {
        kAprilTagMap.put(4, kAprilTag4);
        kAprilTagMap.put(5, kAprilTag5);

        }


    }

    public static class Blue {
        public static final HashMap<Integer, AprilTag> kAprilTagMap = new HashMap<>();
        // From field manual: 46.355 cm
        // From CAD: 46.2534 cm
        // From step layout marking: 46.2788 cm
        // Old value: 46.272  cm
        private static final Translation2d kTag8ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag8ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag8ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag7ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag7ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag7ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag6ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag6ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag6ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag4ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag4ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag4ToLeftAlign = new Translation2d(0.77, 0.55);


        static {

        }
    }
}

