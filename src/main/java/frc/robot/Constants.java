package frc.robot; //These are the imports from reefcape FYI

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.util.Units;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// Constants woohoo

public final class Constants {
    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 0;
        public static final double kDEADBAND = 0.1;
    }

    public static final class DriveConstants {
        public static final double kMaxSpeed = 3;

        // PID values for translation (moving).
        public static final double kPTrans = 5;
        public static final double kITrans = 0;
        public static final double kDTrans = 0;// 0.35005;

        // PID values for rotating.
        public static final double kPAngular = 5;
        public static final double kIAngular = 0;
        public static final double kDAngular = 0;

        public static final PPHolonomicDriveController kPathDriveController = new PPHolonomicDriveController(
            new PIDConstants(DriveConstants.kPTrans, DriveConstants.kITrans, DriveConstants.kDTrans), // Translation
                                                                                                      // PID
            new PIDConstants(DriveConstants.kPAngular, DriveConstants.kIAngular,
                    DriveConstants.kDAngular));
    }

}
