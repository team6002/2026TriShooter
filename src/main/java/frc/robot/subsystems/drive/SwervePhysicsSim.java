package frc.robot.subsystems.drive;

import static org.ode4j.ode.OdeHelper.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.ode4j.math.DMatrix3C;
import org.ode4j.math.DQuaternion;
import org.ode4j.math.DQuaternionC;
import org.ode4j.math.DVector3C;
import org.ode4j.ode.*;

public class SwervePhysicsSim {

    private final DWorld world;
    private final DSpace space;
    private final DJointGroup contactGroup;

    private final DBody chassisBody;
    private final DBox chassisGeom;

    // Robot physical params
    private static final double ROBOT_MASS = 38;
    private static final double ROBOT_LENGTH = 0.685;
    private static final double ROBOT_WIDTH = 0.685;
    private static final double ROBOT_HEIGHT = 0.2;

    private static final double MAX_WHEEL_FORCE = 80;
    private static final double MAX_SPEED = 4.0; // m/s

    // Module positions in robot frame (x forward, y left)
    private final double[][] modulePositions = new double[][] {
        {ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0}, // FL
        {ROBOT_LENGTH / 2.0, -ROBOT_WIDTH / 2.0}, // FR
        {-ROBOT_LENGTH / 2.0, ROBOT_WIDTH / 2.0}, // BL
        {-ROBOT_LENGTH / 2.0, -ROBOT_WIDTH / 2.0} // BR
    };

    public SwervePhysicsSim() {
        initODE2(0);

        world = createWorld();
        space = createSimpleSpace();
        contactGroup = createJointGroup();

        world.setGravity(0, 0, -9.81);
        world.setDamping(0, 0.1); // slows rotation

        // Ground plane z = 0
        createPlane(space, 0, 0, 1, 0);

        // Chassis body
        chassisBody = createBody(world);
        DMass mass = createMass();
        mass.setBoxTotal(ROBOT_MASS, ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT);
        chassisBody.setMass(mass);
        chassisBody.setPosition(3.5, 4, ROBOT_HEIGHT / 2.0 + 0.02);

        chassisGeom = createBox(space, ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT);
        chassisGeom.setBody(chassisBody);

        // Walls
        // Right
        createWall(16.5, 0.1, 1, 8.25, 0);

        // Left
        createWall(16.5, 0.1, 1, 8.25, 8);

        // Back
        createWall(0.1, 8, 1, 0, 4);

        // Front
        createWall(0.1, 8, 1, 16.5, 4);

        // Hub
        createWall(1.2, 1.2, 6, 4.5, 4);

        // Trench Left
        createWall(1, 0.5, 1, 4.5, 6.6);

        // Trench Right
        createWall(1, 0.5, 1, 4.5, 1.4);

        // Bump
        // Left Back
        RampBuilder.createRamp(
                world,
                space,
                4, // baseX
                5.5,
                0,
                "x",
                -15.0, // slope angle
                0.564, // length (X)
                1.854, // width (Y)
                0.16 // height (Z)
                );

        // Left Front
        RampBuilder.createRamp(
                world,
                space,
                4.5, // baseX
                5.5,
                0,
                "x",
                15.0, // slope angle
                0.564, // length (X)
                1.854, // width (Y)
                0.16 // height (Z)
                );

        // Right Back
        RampBuilder.createRamp(
                world,
                space,
                4, // baseX
                2.5,
                0,
                "x",
                -15.0, // slope angle
                0.564, // length (X)
                1.854, // width (Y)
                0.16 // height (Z)
                );

        // Right Front
        RampBuilder.createRamp(
                world,
                space,
                4.5, // baseX
                2.5,
                0,
                "x",
                15.0, // slope angle
                0.564, // length (X)
                1.854, // width (Y)
                0.16 // height (Z)
                );

        // // Funny Ramp
        // RampBuilder.createRampX(
        //     world,
        //     space,
        //     8,

        // );
    }

    public class RampBuilder {
        public static DBody createRamp(
                DWorld world,
                DSpace space,
                double baseX, // lowest point X
                double baseY,
                double baseZ, // lowest point Z
                String axis,
                double angleDeg,
                double length,
                double width,
                double height) {
            final double EPS = 0.001; // prevents ground-plane jitter

            // 1. Create body
            DBody body = OdeHelper.createBody(world);

            // 2. Mass FIRST
            DMass m = OdeHelper.createMass();
            m.setBoxTotal(500.0, length, width, height);
            body.setMass(m);

            // 3. Geom + attach
            DGeom geom = OdeHelper.createBox(space, length, width, height);
            geom.setBody(body);

            // 4. Rotate around Y (slope along +X)
            double angle = Math.toRadians(angleDeg);
            DQuaternion q = new DQuaternion();
            OdeMath.dQFromAxisAndAngle(q, 0, 1, 0, angle);
            body.setQuaternion(q);

            // 5. Compute center position so the *lowest point* sits at baseX/baseZ
            double halfL = length / 2.0;
            double halfH = height / 2.0;

            double centerX = baseX + halfL * Math.cos(angle) + halfH * Math.sin(angle);
            double centerY = baseY + halfL * Math.cos(angle) + halfH * Math.sin(angle);
            double centerZ = baseZ + halfL * Math.sin(angle) - halfH * Math.cos(angle);

            switch (axis) {
                case "x":
                    centerY = baseY;
                case "y":
                    centerX = baseX;
            }

            // 6. Add epsilon to avoid ground-plane collision
            centerZ += EPS;

            // 7. Clamp to world bounds (prevents disappearing)
            if (centerZ < EPS) centerZ = EPS;
            if (centerX < EPS) centerX = EPS;

            // 8. Apply corrected position
            body.setPosition(centerX, centerY, centerZ);

            // 9. Freeze the ramp
            body.setLinearVel(0, 0, 0);
            body.setAngularVel(0, 0, 0);
            body.setKinematic();

            return body;
        }
    }

    private void createWall(double L, double W, double H, double x, double y) {
        // Create a box geom for the wall
        DGeom wallRight = OdeHelper.createBox(space, L, W, H);

        // Position it at x = +4
        wallRight.setPosition(x, y, H / 2.0);
    }

    public void step(double dt, SwerveModuleState[] moduleStates) {
        applyModuleForces(moduleStates);

        // Collisions
        space.collide(null, new DGeom.DNearCallback() {
            @Override
            public void call(Object data, DGeom o1, DGeom o2) {

                DContactBuffer contacts = new DContactBuffer(8);
                int n = OdeHelper.collide(o1, o2, 8, contacts.getGeomBuffer());

                // System.out.println(n);

                for (int i = 0; i < n; i++) {
                    DContact contact = contacts.get(i);

                    // No special flags in your ODE version
                    contact.surface.mode = 0;

                    // THIS is friction
                    contact.surface.mu = 2.5; // strong carpet friction

                    // No bounce
                    contact.surface.bounce = 0.0;
                    contact.surface.bounce_vel = 0.0;

                    // Attach the contact joint
                    DJoint c = OdeHelper.createContactJoint(world, contactGroup, contact);
                    c.attach(o1.getBody(), o2.getBody());
                }
            }
        });

        world.quickStep(dt);
        contactGroup.empty();
    }

    private void applyModuleForces(SwerveModuleState[] moduleStates) {
        if (moduleStates == null || moduleStates.length != 4) return;

        DMatrix3C R = chassisBody.getRotation();
        DVector3C pos = chassisBody.getPosition();

        for (int i = 0; i < 4; i++) {
            SwerveModuleState state = moduleStates[i];

            double speedCmd = state.speedMetersPerSecond;

            // Convert command to [-1, 1]
            double demand = clamp(speedCmd / MAX_SPEED, -1.0, 1.0);

            // CONSTANT FORCE MODEL (realistic acceleration)
            double forceMag = MAX_WHEEL_FORCE * demand;

            // Wheel direction in robot frame
            double wheelAngle = state.angle.getRadians();
            double fx_robot = Math.cos(wheelAngle) * forceMag;
            double fy_robot = Math.sin(wheelAngle) * forceMag;

            // Transform to world frame
            double fx_world = R.get(0, 0) * fx_robot + R.get(0, 1) * fy_robot;
            double fy_world = R.get(1, 0) * fx_robot + R.get(1, 1) * fy_robot;

            // Module position in robot frame
            double rx_robot = modulePositions[i][0];
            double ry_robot = modulePositions[i][1];

            // Rotate module offset into world frame
            double rx_world = R.get(0, 0) * rx_robot + R.get(0, 1) * ry_robot;
            double ry_world = R.get(1, 0) * rx_robot + R.get(1, 1) * ry_robot;

            double px = pos.get0() + rx_world;
            double py = pos.get1() + ry_world;
            double pz = pos.get2();

            chassisBody.addForceAtPos(fx_world, fy_world, 0, px, py, pz);
        }

        // ---------------------------------------
        // LINEAR DRAG (sets terminal velocity)
        // ---------------------------------------
        DVector3C vel = chassisBody.getLinearVel();
        double kDrag = MAX_WHEEL_FORCE + 10; // tune this

        double dragFx = -kDrag * vel.get0();
        double dragFy = -kDrag * vel.get1();

        chassisBody.addForce(dragFx, dragFy, 0);

        // ---------------------------------------
        // ANGULAR DRAG (prevents crazy spinning)
        // ---------------------------------------
        DVector3C angVel = chassisBody.getAngularVel();
        double kAngDrag = 5.0; // tune this

        double tX = -kAngDrag * angVel.get0();
        double tY = -kAngDrag * angVel.get1();
        double tZ = -kAngDrag * angVel.get2();

        chassisBody.addTorque(tX, tY, tZ);
    }

    public void setYaw(double yawRadians) {
        DQuaternion q = new DQuaternion();
        OdeMath.dQFromAxisAndAngle(q, 1, 0, 0, yawRadians);
        chassisBody.setQuaternion(q);

        // Optional: stop spinning
        chassisBody.setAngularVel(0, 0, 0);
    }

    public void syncToRealPose(Pose2d realPose) {
        double xReal = realPose.getX();
        double yReal = realPose.getY();
        double yawReal = realPose.getRotation().getRadians();

        DMatrix3C R = chassisBody.getRotation();

        double roll = Math.atan2(R.get(2, 1), R.get(2, 2));
        double pitch = Math.atan2(-R.get(2, 0), Math.sqrt(R.get(2, 1) * R.get(2, 1) + R.get(2, 2) * R.get(2, 2)));

        DQuaternion qNew = rpyToQuat(roll, pitch, yawReal);

        DVector3C pos = chassisBody.getPosition();
        double zSim = pos.get2();

        chassisBody.setQuaternion(qNew);
        chassisBody.setPosition(xReal, yReal, zSim);
    }

    private DQuaternion rpyToQuat(double roll, double pitch, double yaw) {
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);

        DQuaternion q = new DQuaternion();
        q.set0(cy * cp * cr + sy * sp * sr);
        q.set1(cy * cp * sr - sy * sp * cr);
        q.set2(sy * cp * sr + cy * sp * cr);
        q.set3(sy * cp * cr - cy * sp * sr);
        return q;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    public Pose2d getPose2d() {
        DVector3C pos = chassisBody.getPosition();
        DQuaternionC q = chassisBody.getQuaternion();

        double yaw = quaternionToYaw(q);
        return new Pose2d(pos.get0(), pos.get1(), new Rotation2d(yaw));
    }

    public Pose3d getPose3d() {
        DVector3C pos = chassisBody.getPosition();
        // DQuaternionC q = chassisBody.getQuaternion();
        DMatrix3C R = chassisBody.getRotation();

        double roll = Math.atan2(R.get(2, 1), R.get(2, 2));
        double pitch = Math.atan2(-R.get(2, 0), Math.sqrt(R.get(2, 1) * R.get(2, 1) + R.get(2, 2) * R.get(2, 2)));
        double yaw = Math.atan2(R.get(1, 0), R.get(0, 0));

        return new Pose3d(pos.get0(), pos.get1(), pos.get2(), new Rotation3d(roll, pitch, yaw));
    }

    public void setPose(double x, double y, double yaw) {
        chassisBody.setPosition(x, y, ROBOT_HEIGHT / 2 + 0.01);
        setYaw(yaw);

        // Zero linear velocity
        chassisBody.setLinearVel(0, 0, 0);

        // Zero angular velocity
        chassisBody.setAngularVel(0, 0, 0);

        // Clear accumulated forces/torques
        chassisBody.setForce(0, 0, 0);
        chassisBody.setTorque(0, 0, 0);
    }

    private double quaternionToYaw(DQuaternionC q) {
        double w = q.get0();
        double x = q.get1();
        double y = q.get2();
        double z = q.get3();

        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return Math.atan2(siny_cosp, cosy_cosp);
    }

    public void close() {
        contactGroup.destroy();
        space.destroy();
        world.destroy();
        closeODE();
    }
}
