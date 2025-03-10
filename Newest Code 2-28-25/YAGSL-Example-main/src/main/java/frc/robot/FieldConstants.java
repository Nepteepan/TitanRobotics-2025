
// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;



/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  
    // Auto constants
    public static final double X_REEF_ALIGNMENT_P = 3.3;
    public static final double Y_REEF_ALIGNMENT_P = 3.3;
    public static final double ROT_REEF_ALIGNMENT_P = 0.058;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;
    public static final double X_SETPOINT_REEF_ALIGNMENT = -0.5;  // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.22;  // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;



}
