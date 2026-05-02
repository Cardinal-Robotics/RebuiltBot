package frc.robot; //These are the imports from reefcape FYI

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.*;

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
        public static final double kMaxSpeed = 0;

        public static final PPHolonomicDriveController kPathDriveController = new PPHolonomicDriveController(
                // Translation PID
                new PIDConstants(3, 0, 0), // D 0.35005;
                // Rotation PID
                new PIDConstants(5, 0, 0));
    }

}
