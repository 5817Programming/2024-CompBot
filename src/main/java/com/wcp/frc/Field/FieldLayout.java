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
        private static final Translation2d kTag1ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag1ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag1ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag2ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag2ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag2ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag3ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag3ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag3ToLeftAlign = new Translation2d(0.77, 0.55);

        private static final Translation2d kTag5ToCenterAlign = new Translation2d(0.77, 0.0);
        private static final Translation2d kTag5ToRightAlign = new Translation2d(0.77, -0.55);
        private static final Translation2d kTag5ToLeftAlign = new Translation2d(0.77, 0.55);
        public static final AprilTag kAprilTag1 = new AprilTag(
                4,
                1.355852,
                new Pose2d(
                        new Translation2d(16.7,5.53),
                        Rotation2d.fromDegrees(0)
                ),
                true,
                kTag1ToCenterAlign,
                kTag1ToLeftAlign,
                kTag1ToRightAlign
        );


        static {
        kAprilTagMap.put(4, kAprilTag1);
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

