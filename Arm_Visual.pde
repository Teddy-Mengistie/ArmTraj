import controlP5.*;
import java.util.*;

// UI objects
ControlP5 cp5;
Textfield fileName;
Textfield Q1D;
Textfield Q2D;
Textfield Q1DD;
Textfield Q2DD;
ScrollableList dropdownA;
ScrollableList dropdownB;
Textfield Q1;
Textfield Q2;
Textfield WAYPOINT_NAME;
// Saved waypoints
Map<String, ArmState> waypoints = new HashMap<>();
// axis properties
double heightMeters;
double widthMeters;
double heightRadians;
double widthRadians;
Point center;
ArmState origin;
final float PIXEL_TO_METER = 1 / 200.0; // 1 meter per 200 pixels, 1 pixel = 1/200 meters
final double PIXEL_TO_RAD = 1 / (1080/ (2 * PI));

// Robot
final float ROBOT_WIDTH = 0.762 + 2 * 0.08255; // the width of the robot to be drawn
final float ROBOT_HEIGHT = 0.127;
final int TEAM_NUMBER = 449;
final float WHEEL_RADIUS = 0.0508 / PIXEL_TO_METER;
// Arm
final double L1 = 0.8128;
final double L2 = 0.8636;
Arm arm = new Arm(L1, L2);
final double distFromBase = 0.117475; // m
final double distFromCenter = ROBOT_WIDTH / 2.0 - 0.1524; // m
Point armRoot = new Point(-distFromCenter, distFromBase);

//////////////////////////// Colors
int red = color(255, 0, 0);
int green = color(0, 255, 0);
int blue = color(0, 0, 255);
int teal = color(0, 217, 184);
//////////////// Path config
Path path;
ArmState start = fromDegrees(0, 0);
ArmState a1 = fromDegrees(50, 60);
ArmState a2 = fromDegrees(40, 60);
ArmState end = fromDegrees(70, -20);
/////////////// Trajectory config
Trajectory traj;
boolean showTraj = false;
long startTime = System.currentTimeMillis();
long prevTime = System.currentTimeMillis();
// CONSTRAINTS
double frameperim = 1.22; // meters
double heightLimit = 1.98; // meters
double maxq1dot = 6.0; // rad/s
double maxq2dot = 6.0; // rad/s
double maxq1ddot = 5.0; // rad/s/s
double maxq2ddot = 5.0; // rad/s/s
////////////////////////////
// Bad Points
List<ArmState> badPoints = new ArrayList<>();

void setup() {
  fullScreen();
  frameRate(160);

  // axis generation
  heightMeters = displayHeight * PIXEL_TO_METER;
  widthMeters = displayWidth * PIXEL_TO_METER;
  heightRadians = displayHeight * PIXEL_TO_RAD;
  widthRadians = displayWidth * PIXEL_TO_RAD;
  center = new Point(widthMeters / 2, heightMeters/2);
  origin = new ArmState(0.0, heightRadians/2);

  // path and trajectory initiation
  path = new Path(start, a1, a2, end);
  traj = new Trajectory(maxq1dot, maxq2dot, maxq1ddot, maxq2ddot, path);

  // construct UI
  cp5 = new ControlP5(this);

  // Default tab to be renamed to trajectory tab instead
  cp5.getTab("default").setLabel("trajectory");

  // flip switch for traj
  cp5.addToggle("show")
    .setPosition(10, height - 40)
    .setSize(50, 20)
    .setValue(false)
    .setMode(ControlP5.SWITCH);

  // ****** text boxes for changing the constraints ******

  Q1D = cp5.addTextfield("max q1 vel")
    .setPosition(10, height - 80)
    .setSize(100, 20)
    .moveTo("constraints")
    .setValue(maxq1dot + "")
    .setAutoClear(false)
    .setInputFilter(2);

  Q2D = cp5.addTextfield("max q2 vel")
    .setPosition(10, height - 40)
    .setSize(100, 20)
    .moveTo("constraints")
    .setValue(maxq2dot + "")
    .setAutoClear(false)
    .setInputFilter(2);

  Q1DD = cp5.addTextfield("max q1 accel")
    .setPosition(120, height - 80)
    .setSize(100, 20)
    .moveTo("constraints")
    .setValue(maxq2ddot + "")
    .setAutoClear(false)
    .setInputFilter(2);

  Q2DD = cp5.addTextfield("max q2 accel")
    .setPosition(120, height - 40)
    .setSize(100, 20)
    .moveTo("constraints")
    .setValue(maxq2ddot + "")
    .setAutoClear(false)
    .setInputFilter(2);

  // Text boxes for adding/updating a point
  Q1 = cp5.addTextfield("q1")
    .setPosition(10, height - 80)
    .setSize(100, 20)
    .moveTo("Points")
    .setValue(path.start.q1 + "")
    .setAutoClear(false)
    .setInputFilter(3);

  Q2 = cp5.addTextfield("q2")
    .setPosition(10, height - 40)
    .setSize(100, 20)
    .moveTo("Points")
    .setValue(path.start.q2 + "")
    .setAutoClear(false)
    .setInputFilter(3);

  WAYPOINT_NAME = cp5.addTextfield("name")
    .setPosition(120, height - 80)
    .setSize(100, 20)
    .setInputFilter(3)
    .moveTo("Points");

  // Button to add/update a point
  cp5.addBang("update")
    .setPosition(120, height - 40)
    .setSize(100, 20)
    .setLabel("Add/update")
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER)
    .moveTo("Points");

  // ****** dropdown for point a ******
  dropdownA = cp5.addScrollableList("A")
    .setPosition(width - 230, 10)
    .setSize(100, 100)
    .setItemHeight(20)
    .setType(ControlP5.DROPDOWN)
    .setBarHeight(20);

  // ****** dropdown for point b ******
  dropdownB = cp5.addScrollableList("B")
    .setPosition(width - 120, 10)
    .setSize(100, 100)
    .setItemHeight(20)
    .setType(ControlP5.DROPDOWN)
    .setBarHeight(20);

  // file name field
  fileName = cp5.addTextfield("fileName")
    .setPosition(10, height - 80)
    .setSize(100, 20)
    .setLabel("Name")
    .setAutoClear(false)
    .setInputFilter(0);
  
  // button to save to json file
  cp5.addBang("save")
    .setPosition(120, height - 80)
    .setSize(50, 20)
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER);

  // button to load a json file
  cp5.addBang("load")
    .setPosition(180, height - 40)
    .setSize(80, 20)
    .setTriggerEvent(Bang.RELEASE)
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER);

  cp5.addBang("reverse")
    .setPosition(180, height - 80)
    .setSize(80, 20)
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER);

  // button to generate the trajectory from the path
  cp5.addBang("generate")
    .setPosition(70, height - 40)
    .setSize(100, 20)
    .setLabel("Generate Trajectory")
    .align(ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER, ControlP5.CENTER);

  //  dropdown to choose start
  // add option to have a clear UI
  cp5.addTab("Clear UI");
  
  // Scan for bad points where the arm should not go
  //for (float q1 = 0.0; q1 <= PI; q1 += .03) {
  //  for (float q2 = -PI; q2 <= PI; q2 += .03) {
  //    var state = new ArmState(q1, q2);
  //    if (state.violatesConstraints()) badPoints.add(state);
  //  }
  //}
  
}
////////////////
public void drawRobot() {
  noFill();
  Point topLeft = new Point(-heightLimit, heightLimit - 0.04064 - ROBOT_HEIGHT);
  Point topRight = new Point(heightLimit, heightLimit - 0.04064 - ROBOT_HEIGHT);
  topLeft.lineTo(topRight);
  
  Point corner1 = new Point(-ROBOT_WIDTH / 2, 0.0).asPixel();
  fill(red);
  rect(corner1.x, corner1.y, ROBOT_WIDTH / PIXEL_TO_METER, ROBOT_HEIGHT / PIXEL_TO_METER, 7.0);
  fill(255);
}

class Point {
  float x; // x coordinate meters or pixels
  float y; // y coordinate meters or pixels

  public Point (double x, double y) {
    this.x = (float)x;
    this.y = (float)y;
  }

  public Point plus(Point b) {
    return new Point (x + b.x, y + b.y);
  }

  public Point minus(Point b) {
    return new Point(x - b.x, y - b.y);
  }

  public Point times(double k) {
    return new Point(x * k, y * k);
  }

  public Point asPixel() {
    double pixelX = (x + center.x) / PIXEL_TO_METER;
    double pixelY = (-y + center.y) / PIXEL_TO_METER;
    return new Point(pixelX, pixelY);
  }

  public Point asPoint() {
    double pointX = x * PIXEL_TO_METER - center.x;
    double pointY = -y * PIXEL_TO_METER + center.y;
    return new Point(pointX, pointY);
  }
  // treating point as vector
  public double mag() {
    return sqrt(x * x + y * y);
  }

  public void lineTo(Point p) {
    stroke(255);
    Point from = this.asPixel();
    Point to = p.asPixel();
    line(from.x, from.y, to.x, to.y);
  }

  public void show() {
    noStroke();
    fill(255);
    Point center = this.asPixel();
    circle(center.x, center.y, 10.0);
  }

  public void show(float radius) {
    noStroke();
    fill(255);
    Point center = this.asPixel();
    circle(center.x, center.y, radius);
  }
  public void show(float radius, int col) {
    noStroke();
    fill(col);
    Point center = this.asPixel();
    circle(center.x, center.y, radius);
  }

  @Override
    public String toString() {
    return "(" + x + ", " + y + ")";
  }
}

class Arm {
  double l1;
  double l2;
  ArmState currentState = new ArmState(0.0, 0.0);

  public Arm(double l1, double l2) {
    this.l1 = l1;
    this.l2 = l2;
  }

  // show only end effector of this state
  public void showEndEffector(ArmState state, int col) {
    double c1 = Math.cos(state.q1);
    double s1 = Math.sin(state.q1);
    double c12 = Math.cos(state.q1 + state.q2);
    double s12 = Math.sin(state.q1 + state.q2);
    new Point(l1 * c1 + l2 * c12, l1 * s1 + l2 * s12).plus(armRoot).show(2, col);
  }

  // Show the arm using some geometry (forward kinematics)
  public void show() {
    double c1 = Math.cos(currentState.q1);
    double s1 = Math.sin(currentState.q1);
    double c12 = Math.cos(currentState.q1 + currentState.q2);
    double s12 = Math.sin(currentState.q1 + currentState.q2);
    Point endEffector = new Point(l1 * c1 + l2 * c12, l1 * s1 + l2 * s12).plus(armRoot);
    Point joint = new Point(l1 * c1, l1 * s1).plus(armRoot);
    armRoot.lineTo(joint);
    joint.lineTo(endEffector);
    endEffector.show(10, red);
    joint.show(10, red);
    armRoot.show(20, green);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void draw() {
  long currentTime = System.currentTimeMillis();
  double t = (currentTime - startTime) / 1000.0;
  double dt = (currentTime - prevTime) / 1000.0;
  background(0);
  drawRobot();
  // show the trajectory and path so that it can be modified
  if (showTraj) {
    for (ArmState point : badPoints) point.show(1, red);
    path.showPath();
    traj.show();
    path.show();
  }
  arm.show();
  // use velocity instead
  var sample = traj.sample(t % traj.totalTimeSeconds);
  var dv = new ArmState(sample.q1dot, sample.q2dot).times(dt);
  arm.currentState = sample.plus(dv);
  prevTime = currentTime;
}

// UNDER THIS IS FOR THE TRAJ EDITOR AND GENERATOR
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Joint space path
class Path {
  ArmState start;
  ArmState a1; // anchor point 1
  ArmState a2; // anchor point 2
  ArmState end;

  public Path (ArmState start, ArmState a1, ArmState a2, ArmState end) {
    this.start = start;
    this.a1 = a1;
    this.a2 = a2;
    this.end = end;
  }

  /**
   * @param t sample cubic bezier based on this internal parameter from 0 to 1
   */
  public ArmState sample(double t) {
    return bezier(start, a1, a2, end, t);
  }
  /**
   * @param ds the space between each point along the path to be satisfied
   */
  public List<ArmState> collectSamples(int n) {
    double dt = 1.0/n;
    double distance = 0.0;
    List<ArmState> result = new ArrayList<>();
    ArmState prevState = sample(0.0);
    for (double t = 0; t <= 1.0; t += dt) {
      ArmState currState = sample(t);
      var diff = currState.minus(prevState);
      distance += diff.mag();
      currState.s = distance;
      currState.v_theta = diff.getAngle();
      result.add(currState); // move on to the next length
      prevState = currState;
    }
    return result;
  }

  public void showPath() {
    sample(0.0).show(6, green);
    for (double t = .02; t < 1.0; t += .02) {
      ArmState pt = sample(t);
      pt.show(3, color(255, 255, 255, 50));
      arm.showEndEffector(pt, color(255, 255, 255));
    }
    sample(1.0).show(6, red);
  }

  public void show() {
    start.lineTo(a1);
    end.lineTo(a2);
    start.show(12, green);
    a1.show(12);
    a2.show(12);
    end.show(12, red);
  }
}

class Trajectory {
  double maxq1dot, maxq2dot, maxq1ddot, maxq2ddot;
  double totalTimeSeconds;
  List<ArmState> points;
  Path path;
  public Trajectory (double maxq1dot, double maxq2dot, double maxq1ddot, double maxq2ddot, Path path) {
    this.maxq1dot = maxq1dot;
    this.maxq2dot = maxq2dot;
    this.maxq1ddot = maxq1ddot;
    this.maxq2ddot = maxq2ddot;
    this.path = path;
    parametrizeTrajectory();
  }

  public ArmState sample(double t) {
    int n = points.size();
    for (int i = 1; i < n; i++) {
      ArmState curr = points.get(i);
      ArmState prev = points.get(i-1);
      if (curr.t == t) return curr;
      if (curr.t > t) {
        // interpolate between prev and current
        ArmState diff = curr.minus(prev);
        ArmState prevV = new ArmState(prev.q1dot, prev.q2dot);
        ArmState vDiff = new ArmState(curr.q1dot, curr.q2dot).minus(prevV);
        double t_err = t - prev.t;
        double dt = curr.t - prev.t;
        double k = t_err / dt;
        ArmState qdot = prevV.plus(vDiff.times(k));
        ArmState q = prev.plus(diff.times(k));
        q.q1dot = qdot.q1dot;
        q.q2dot = qdot.q2dot;
        return q;
      }
    }
    return points.get(n-1);
  }

  public void updateConstraints() {
    maxq1dot = Double.parseDouble(Q1D.getText());
    maxq2dot = Double.parseDouble(Q2D.getText());
    maxq1ddot = Double.parseDouble(Q1DD.getText());
    maxq2ddot = Double.parseDouble(Q2DD.getText());
  }
  // O(n) n = number of points sampled
  public void parametrizeTrajectory() {

    // collect samples from path to use for generating trajectory
    points = path.collectSamples(1000);

    try {
      updateConstraints();
    }
    catch(Exception e) {
      // do nothing
    }

    double maxV = Math.sqrt(maxq1dot * maxq1dot + maxq2dot * maxq2dot);
    int n = points.size();
    // ***************************
    // parametrize trajectory here

    // Step 0: set every velocity to the maximum that follows all constraints
    for (ArmState point : points) {
      point.v = maxV;
      point.step = 0;
    }

    // Step 1: Apply curvature constraints,
    // Allows the arm to slow down on tight changes in direction of v
    for (int i = 1; i < n; i++) {
      ArmState currPt = points.get(i);
      ArmState prevPt = points.get(i-1);
      double ds = Math.abs(currPt.s - prevPt.s);
      double dtheta = currPt.v_theta - prevPt.v_theta;
      // wrap around dtheta
      if (dtheta > PI)
        dtheta -= 2 * PI;
      else if (dtheta < -PI)
        dtheta += 2 * PI;
      double curvature = Math.abs(dtheta / ds);
      double sa = Math.abs(Math.sin(currPt.v_theta));
      double ca = Math.abs(Math.cos(currPt.v_theta));
      if ((currPt.v * currPt.v * curvature * sa) > maxq1ddot) {
        currPt.step = 1;
        currPt.v = Math.min(currPt.v, Math.sqrt(maxq1ddot / sa / curvature));
      }
      if ((currPt.v * currPt.v * curvature * ca) > maxq2ddot) {
        currPt.step = 1;
        currPt.v = Math.min(currPt.v, Math.sqrt(maxq2ddot / ca / curvature));
      }
    }

    // Step 2: Forward pass, start with v as 0.0
    // Use equation vf = sqrt(v0^2 + 2ad)
    points.get(0).v = 0.0; // start at v = 0
    for (int i = 1; i < n; i++) {
      ArmState currPt = points.get(i);
      ArmState prevPt = points.get(i-1);
      double ds = Math.abs(currPt.s - prevPt.s);
      double ctheta = Math.abs((Math.cos(currPt.v_theta)));
      double stheta = Math.abs((Math.sin(currPt.v_theta)));
      double maxA = (ctheta * maxq1ddot + stheta * maxq2ddot);
      double vi = prevPt.v;
      if (Math.sqrt(vi * vi + 2.0 * maxA * ds) <= currPt.v) currPt.step = 2;
      double vf = Math.min(currPt.v, Math.sqrt(vi * vi + 2.0 * maxA * ds));
      currPt.v = vf;
    }

    // Step 3: Backward pass
    // Use equation vf = sqrt(v0^2 + 2ad)
    points.get(n-1).v = 0.0; // start at v = 0
    for (int i = n-2; i >= 0; i--) {
      ArmState currPt = points.get(i);
      ArmState prevPt = points.get(i+1);
      double ds = Math.abs(currPt.s - prevPt.s);
      double ctheta = Math.abs((Math.cos(currPt.v_theta)));
      double stheta = Math.abs((Math.sin(currPt.v_theta)));
      double maxA = (ctheta * maxq1ddot + stheta * maxq2ddot);
      double vi = prevPt.v;
      if (Math.sqrt(vi * vi + 2.0 * maxA * ds) <= currPt.v) currPt.step = 3;
      double vf = Math.min(currPt.v, Math.sqrt(vi * vi + 2.0 * maxA * ds));
      currPt.v = vf;
      currPt.q1dot = (float)(currPt.v * Math.cos(currPt.v_theta));
      currPt.q2dot = (float)(currPt.v * Math.sin(currPt.v_theta));
    }
    // ***************************
    // Step 4: find t at each point
    points.get(0).t = 0;
    for (int i = 1; i < points.size(); i++) {
      ArmState curr = points.get(i);
      ArmState prev = points.get(i-1);
      double ds = curr.s - prev.s;
      double dt = 2 * ds / (curr.v + prev.v);
      curr.t = (float)(prev.t + dt);
      totalTimeSeconds = curr.t;
    }
  }

  public void save(String fileName) {
    JSONObject object = new JSONObject();
    
    // gather points into array
    JSONArray array = new JSONArray();
    for (int i = 0; i < points.size(); i++) {
      JSONObject currState = new JSONObject();
      ArmState state = points.get(i);
      currState.setFloat("t", (float)state.t);
      currState.setFloat("q1", state.q1);
      currState.setFloat("q2", state.q2);
      currState.setFloat("q1d", state.q1dot);
      currState.setFloat("q2d", state.q2dot);
      array.setJSONObject(i, currState);
    }
    object.setJSONArray("states", array);

    // save info about this path for reverse engineering
    JSONObject metadata = new JSONObject();
    metadata.setFloat("start q1", path.start.q1);
    metadata.setFloat("start q2", path.start.q2);
    metadata.setFloat("anchor1 q1", path.a1.q1);
    metadata.setFloat("anchor1 q2", path.a1.q2);
    metadata.setFloat("anchor2 q1", path.a2.q1);
    metadata.setFloat("anchor2 q2", path.a2.q2);
    metadata.setFloat("end q1", path.end.q1);
    metadata.setFloat("end q2", path.end.q2);
    object.setJSONObject("metadata", metadata);
    
    // save to filename.json
    saveJSONObject(object, fileName + ".json");
  }
  public void show() {
    for (ArmState pt : points) {
      switch(pt.step) {
      case 0:
        pt.show((float)pt.v + .2, color(255));
        break;
      case 1:
        pt.show((float)pt.v + .2, green);
        break;
      case 2:
        pt.show((float)pt.v + .2, blue);
        break;
      case 3:
        pt.show((float)pt.v + .2, red);
        break;
      default:
        pt.show((float)pt.v + .2, color(0, 255, 0));
      }

      arm.showEndEffector(pt, color(255));
    }
  }
}

public ArmState bezier(ArmState start, ArmState a1, ArmState a2, ArmState end, double t) { // bel
  ArmState first = start.times(Math.pow((1-t), 3));
  ArmState second = a1.times(3 * Math.pow((1-t), 2) * t);
  ArmState third = a2.times(3 * (1-t) * t * t);
  ArmState fourth = end.times(t * t * t);
  return first.plus(second).plus(third).plus(fourth);
}

class ArmState {
  float q1;
  float q2;
  float q1dot;
  float q2dot;
  double s; // distance along path
  double v_theta;
  double v;
  float t;
  int step;

  public ArmState(double q1, double q2) {
    this.q1 = (float) q1;
    this.q2 = (float) q2;
  }

  public ArmState plus(ArmState b) {
    return new ArmState (q1+ b.q1, q2 + b.q2);
  }

  public ArmState minus(ArmState b) {
    return new ArmState(q1- b.q1, q2 - b.q2);
  }

  public ArmState times(double k) {
    return new ArmState(q1* k, q2* k);
  }

  public double getAngle() {
    return atan2(q2, q1);
  }
  
  public double getY() {
    return L1 * sin(q1) + L2 * sin(q1 + q2);  
  }
  
  public double getX() {
    return L1 * cos(q1) + L2 * cos(q1 + q2); 
  }
  
  public boolean violatesConstraints() {
    return true;  
  }
  
  public ArmState asPixel() {
    double pixelX = (q1 + origin.q1) / PIXEL_TO_RAD;
    double pixelY = (-q2 + origin.q2) / PIXEL_TO_RAD;
    return new ArmState(pixelX, pixelY);
  }

  public ArmState asPoint() {
    double pointX = q1 * PIXEL_TO_RAD - origin.q1;
    double pointY = -q2 * PIXEL_TO_RAD + origin.q2;
    return new ArmState(pointX, pointY);
  }
  // treating point as vector
  public double mag() {
    return sqrt(q1 * q1 + q2 * q2);
  }

  public void lineTo(ArmState p) {
    stroke(255);
    ArmState from = this.asPixel();
    ArmState to = p.asPixel();
    line(from.q1, from.q2, to.q1, to.q2);
  }

  public void show() {
    noStroke();
    fill(255);
    ArmState center = this.asPixel();
    circle(center.q1, center.q2, 10.0);
  }

  public void show(float radius) {
    noStroke();
    fill(255);
    ArmState center = this.asPixel();
    circle(center.q1, center.q2, radius);
  }

  public void show(float radius, int col) {
    noStroke();
    fill(col);
    ArmState center = this.asPixel();
    circle(center.q1, center.q2, radius);
  }

  @Override
    public String toString() {
    return "(" + q1+ ", " + q2 + ")";
  }
}

// UTIL FUNCTIONS
public ArmState fromDegrees(float theta, float beta) {
  return new ArmState(toRadians(theta), toRadians(beta));
}

public float toRadians(float degree) {
  return degree * PI / 180;
}

// called by toggle
void show(boolean flag) {
  showTraj = !flag;
}

// add/update the list of waypoints saved
void update() {
  try {
    var q1 = Float.parseFloat(Q1.getText());
    var q2 = Float.parseFloat(Q2.getText());
    var name = WAYPOINT_NAME.getText();
    if (name.equals("")) return;
    ArmState pt = fromDegrees(q1, q2);
    waypoints.putIfAbsent(name, pt);
    waypoints.replace(name, pt);
    dropdownA.removeItem(name);
    dropdownB.removeItem(name);
    dropdownA.addItem(name, pt);
    dropdownB.addItem(name, pt);
  }
  catch(Exception e) {
    System.err.println("ERROR: Values entered were not numbers.");
  }
}

void A(int n) {
  var item = dropdownA.getItem(n);
  path.start = waypoints.get(item.get("name"));
}

void B(int n) {
  var item = dropdownB.getItem(n);
  path.end = waypoints.get(item.get("name"));
}

void updatePathFromFile(File file) {
  if (file == null) {
    System.err.println("NULL FILE : User did not select a file.");
  } else {
    try {
      // parse the settings of the path
      JSONObject json = loadJSONObject(file);
      JSONObject metadata = json.getJSONObject("metadata");
      path.start.q1 = metadata.getFloat("start q1");
      path.start.q2 = metadata.getFloat("start q2");
      path.a1.q1 = metadata.getFloat("anchor1 q1");
      path.a1.q2 = metadata.getFloat("anchor1 q2");
      path.a2.q1 = metadata.getFloat("anchor2 q1");
      path.a2.q2 = metadata.getFloat("anchor2 q2");
      path.end.q1 = metadata.getFloat("end q1");
      path.end.q2 = metadata.getFloat("end q2");
      fileName.setText(file.getName().replace(".json", ""));
    }
    catch(Exception e) {
      // failed to load, dont update current path
      return;
    }
  }
}

void load() {
  selectInput("Select a path to modify", "updatePathFromFile");
}

void save() {
  String file = fileName.getText();
  if (file.contains(".")) return; // trying to save
  traj.save(file);
}

/** method to reverse the path */
void reverse() {
  // swap start and end
  var pointer = path.start;
  path.start = path.end;
  path.end = pointer;
  // swap anchor points
  pointer = path.a1;
  path.a1 = path.a2;
  path.a2 = pointer;
}

void generate() {
  traj.parametrizeTrajectory();
}

//////////////////////////
// Mouse functionality ///
//////////////////////////

boolean draggingA = false;
boolean draggingB = false;
boolean draggingStart = false;
boolean draggingEnd = false;

void mousePressed() {
  if (new ArmState(mouseX, mouseY).minus(path.a1.asPixel()).mag() <= 6.0) {
    draggingA = true;
  } else if (new ArmState(mouseX, mouseY).minus(path.a2.asPixel()).mag() <= 6.0) {
    draggingB = true;
  } else if (new ArmState(mouseX, mouseY).minus(path.start.asPixel()).mag() <= 6.0) {
    draggingStart = true;
  } else if (new ArmState(mouseX, mouseY).minus(path.end.asPixel()).mag() <= 6.0) {
    draggingEnd = true;
  }
}

void mouseDragged() {
  if (draggingA) path.a1 = new ArmState(mouseX, mouseY).asPoint();
  else if (draggingB) path.a2 = new ArmState(mouseX, mouseY).asPoint();
  else if (draggingStart) path.start = new ArmState(mouseX, mouseY).asPoint();
  else if (draggingEnd) path.end = new ArmState(mouseX, mouseY).asPoint();
}

void mouseReleased() {
  draggingA = false;
  draggingB = false;
  draggingStart = false;
  draggingEnd = false;
}
