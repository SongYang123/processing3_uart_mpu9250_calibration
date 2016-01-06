import processing.serial.*;

void setup() {
    size(640, 360, P3D);
    Coord.init(this);
    Cam.init(this);
    Com.init(this, 3);
}

void draw() {
    background(0);

    Coord.drawCoord();

    strokeWeight(2);
    stroke(255);
    if (Com.getReady()) {
        int[] data = Com.getData();
        Com.clear();
        Calibration.push(data[0], data[1], data[2]);
    }
    float[][] pointBuf = Calibration.getPtBuf();
    for (int i = 0; i < pointBuf.length; i++) {
        point(pointBuf[i][0], pointBuf[i][1], pointBuf[i][2]);
    }

    strokeWeight(6);
    stroke(255, 0, 255);
    float[] centre = Calibration.getCentre();
    if (centre != null) {
        point(centre[0], centre[1], centre[2]);
        //print(centre[0], centre[1], centre[2]);
    }
}

private static class Calibration {
    private static final int SAMPLE_MIN = 60;
    private static final float DIST_MIN = 10.0;
    private static float[][] pt = new float[SAMPLE_MIN][3];
    private static int ptIdx = 0;
    public static void push(float x, float y, float z) {
        for (int i = 0; i < SAMPLE_MIN; i++) {
            if (dist(x, y, z, pt[i][0], pt[i][1], pt[i][2]) < DIST_MIN) {
                return;
            }
        }
        pt[ptIdx][0] = x;
        pt[ptIdx][1] = y;
        pt[ptIdx][2] = z;
        if (ptIdx == SAMPLE_MIN - 1) {
            ptIdx = 0;
        } else {
            ptIdx++;
        }
    }
    public static float[][] getPtBuf() {
        return pt;
    }
    public static float[] getCentre() {
        float[] avgPt = calAvgPt();
        float[][] norPt = calNorPt(avgPt);
        return calCentre(calLhsInv(calLhs(norPt)), calRhs(norPt), avgPt);
    }
    private static float[] calAvgPt() {
        float[] avg = new float[3];
        for (int i = 0; i < SAMPLE_MIN; i++) {
            for (int j = 0; j < 3; j++) {
                avg[j] += pt[i][j];
            }
        }
        for (int j = 0; j < 3; j++) {
            avg[j] /= SAMPLE_MIN;
        }
        return avg;
    }
    private static float[][] calNorPt(float[] avg) {
        float[][] norPt = new float[SAMPLE_MIN][3];
        for (int i = 0; i < SAMPLE_MIN; i++) {
            for (int j = 0; j < 3; j++) {
                norPt[i][j] = pt[i][j] - avg[j];
            }
        }
        return norPt;
    }
    private static float[] calRhs(float[][] norPt) {
        float[] rhs = new float[3];
        float[] norPtSqSum = new float[SAMPLE_MIN];
        for (int i = 0; i < SAMPLE_MIN; i++) {
            norPtSqSum[i] = sq(norPt[i][0]) + sq(norPt[i][1]) + sq(norPt[i][2]);
        }
        for (int i = 0; i < SAMPLE_MIN; i++) {
            for (int j = 0; j < 3; j++) {
                rhs[j] += norPt[i][j] * norPtSqSum[i];
            }
        }
        for (int j = 0; j < 3; j++) {
            rhs[j] /= 2;
        }
        return rhs;
    }
    private static float[][] calLhs(float[][] norPt) {
        float[][] lhs = new float[3][3];
        for (int i = 0; i < SAMPLE_MIN; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    lhs[j][k] += norPt[i][j] * norPt[i][k];
                }
            }
        }
        return lhs;
    }
    private static float det2(float m00, float m01, float m10, float m11) {
        return m00 * m11 - m01 * m10;
    }
    private static float[][] calLhsInv(float[][] lhs) {
        float inv[][] = new float[3][3];
        float det = lhs[0][0] * det2(lhs[1][1], lhs[1][2], lhs[2][1], lhs[2][2]);
        det -= lhs[0][1] * det2(lhs[1][0], lhs[1][2], lhs[2][0], lhs[2][2]);
        det += lhs[0][2] * det2(lhs[1][0], lhs[1][1], lhs[2][0], lhs[2][1]);
        if (det == 0) {
            return null;
        }
        inv[0][0] = det2(lhs[1][1], lhs[1][2], lhs[2][1], lhs[2][2]) / det;
        inv[0][1] = det2(lhs[0][2], lhs[0][1], lhs[2][2], lhs[2][1]) / det;
        inv[0][2] = det2(lhs[0][1], lhs[0][2], lhs[1][1], lhs[1][2]) / det;
        inv[1][0] = det2(lhs[1][2], lhs[1][0], lhs[2][2], lhs[2][0]) / det;
        inv[1][1] = det2(lhs[0][0], lhs[0][2], lhs[2][0], lhs[2][2]) / det;
        inv[1][2] = det2(lhs[0][2], lhs[0][0], lhs[1][2], lhs[1][0]) / det;
        inv[2][0] = det2(lhs[1][0], lhs[1][1], lhs[2][0], lhs[2][1]) / det;
        inv[2][1] = det2(lhs[0][1], lhs[0][0], lhs[2][1], lhs[2][0]) / det;
        inv[2][2] = det2(lhs[0][0], lhs[0][1], lhs[1][0], lhs[1][1]) / det;
        return inv;
    }
    private static float[] calCentre(float[][] lhsInv, float[] rhs, float[] avg) {
        float[] centre = new float[3];
        if (lhsInv == null) {
            return null;
        }
        for (int i = 0; i < 3; i++) {
            centre[i] = avg[i];
            for (int j = 0; j < 3; j++) {
                centre[i] += lhsInv[i][j] * rhs[j];
            }
        }
        return centre;
    }
}

void mousePressed() {
    Cam.onMousePressed();
}

void mouseDragged() {
    Cam.onMouseDragged();
}

void mouseWheel(MouseEvent event) {
    Cam.onMouseWheel(event);
}

void serialEvent(Serial port)
{
    Com.onSerialEvent(port);
}

private static class Coord {

    private static final float AXIS_LEN = 200.0;
    private static final float DASH_DIST = 10.0;

    private static PApplet applet;

    public static void init(PApplet app) {
        applet = app;
    }

    public static void drawCoord() {
        applet.strokeWeight(1);
        applet.stroke(255, 0, 0);
        applet.line(0.0, 0.0, 0.0, AXIS_LEN, 0.0, 0.0);
        dashedLine(0.0, 0.0, 0.0, -AXIS_LEN, 0.0, 0.0);
        applet.stroke(0, 255, 0);
        applet.line(0.0, 0.0, 0.0, 0.0, AXIS_LEN, 0.0);
        dashedLine(0.0, 0.0, 0.0, 0.0, -AXIS_LEN, 0.0);
        applet.stroke(0, 0, 255);
        applet.line(0.0, 0.0, 0.0, 0.0, 0.0, AXIS_LEN);
        dashedLine(0.0, 0.0, 0.0, 0.0, 0.0, -AXIS_LEN);
    }

    private static void dashedLine(float x1, float y1, float z1, float x2, float y2, float z2) {
        boolean dashed = true;
        float index;
        PVector v = new PVector(x2 - x1, y2 - y1, z2 - z1);
        float len = v.mag();
        v = v.normalize();
        for (index = 0.0; index < len - DASH_DIST; index += DASH_DIST) {
            if (dashed) {
                float endIndex = index + DASH_DIST;
                applet.line(x1+index*v.x, y1+index*v.y, z1+index*v.z, x1+endIndex*v.x, y1+endIndex*v.y, z1+endIndex*v.z);
                dashed = false;
            } else {
                dashed = true;
            }
        }
        if (dashed) {
            applet.line(x1+index*v.x, y1+index*v.y, z1+index*v.z, x2, y2, z2);
        }
    }
}

private static class Cam {

    public static final float FOV = PI / 3.0;
    private static final float ANGLE_X_MAX = PI * 5.0 / 12.0;
    private static final float LEN_MIN = 100.0;
    private static final float LEN_MAX = 1000.0;
    private static final float ZOOM_FACTOR = 20.0;

    private static PApplet applet;
    private static float ratio;
    private static float[][] mat;
    private static float len;
    private static int prevMouseX;
    private static int prevMouseY;

    public static void init(PApplet app) {
        applet = app;
        ratio = (float) app.width / app.height;
        mat = Mat.getIdentical();
        len = (app.height/2.0) / tan(FOV/2.0);
    }

    public static void onMousePressed() {
        prevMouseX = applet.mouseX;
        prevMouseY = applet.mouseY;
    }

    public static void onMouseDragged() {
        float ax = PI * (applet.mouseY - prevMouseY) / applet.height;
        float ay = -PI * (applet.mouseX - prevMouseX) / applet.height;
        float[][] temp = Mat.multiply(mat, Mat.getRotateX(ax));
        temp = Mat.multiply(Mat.getRotateY(ay), temp);
        float angle = atan2(-temp[1][2], temp[1][1]);
        if (angle < ANGLE_X_MAX && angle > -ANGLE_X_MAX) {
            mat = temp;
            applet.camera(mat[0][2]*len, mat[1][2]*len, mat[2][2]*len, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        }
        prevMouseX = applet.mouseX;
        prevMouseY = applet.mouseY;
    }

    public static void onMouseWheel(MouseEvent event) {
        float delta = event.getCount() * ZOOM_FACTOR;
        float temp = len;
        len = constrain(len + delta, LEN_MIN, LEN_MAX);
        if (len != temp) {
            applet.perspective(Cam.FOV, ratio, len/10.0, len*10.0);
            applet.camera(mat[0][2]*len, mat[1][2]*len, mat[2][2]*len, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        }
    }
}

private static class Com {

    private static final int BAUD_RATE = 9600;

    private static Serial port;
    private static boolean ready = false;
    private static byte[] rx;
    private static int[] data;

    public static void init(PApplet app, int size) {
        port = new Serial(app, Serial.list()[2], BAUD_RATE);
        port.buffer(size * 2);
        rx = new byte[size * 2];
        data = new int[size];
    }

    public static boolean getReady() {
        if (ready) {
            ready = false;
            return true;
        } else {
            return false;
        }
    }

    public static int[] getData() {
        return data;
    }

    public static void clear() {
        port.clear();
    }

    public static void onSerialEvent(Serial port) {
        port.readBytes(rx);
        for (int i = 0; i < data.length; i++) {
            data[i] = byteToInt16(rx[i * 2], rx[i * 2 + 1]);
        }
        ready = true;
        //port.clear();
    }

    private static int byteToInt16(byte lsb, byte msb) {
        int result;
        result = ((msb & 0xFF) << 8) | (lsb & 0xFF);
        if (result > 32768) {
            return result - 65536;
        } else {
            return result;
        }
    }
}

private static class Mat {

    public static float[][] getIdentical() {
        float[][] result = new float[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (i == j) {
                    result[i][j] = 1.0;
                } else {
                    result[i][j] = 0.0;
                }
            }
        }
        return result;
    }

    public static float[][] multiply(float[][]lhs, float[][]rhs) {
        float[][] result = new float[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 3; k++) {
                    result[i][j] += lhs[i][k] * rhs[k][j];
                }
            }
        }
        return result;
    }

    public static float[][] getRotateX(float angle) {
        float[][]result = new float[3][3];
        float s = sin(angle);
        float c = cos(angle);
        result[0][0] = 1.0;
        result[0][1] = 0.0;
        result[0][2] = 0.0;
        result[1][0] = 0.0;
        result[1][1] = c;
        result[1][2] = -s;
        result[2][0] = 0.0;
        result[2][1] = s;
        result[2][2] = c;
        return result;
    }

    public static float[][] getRotateY(float angle) {
        float[][]result = new float[3][3];
        float s = sin(angle);
        float c = cos(angle);
        result[0][0] = c;
        result[0][1] = 0.0;
        result[0][2] = s;
        result[1][0] = 0.0;
        result[1][1] = 1.0;
        result[1][2] = 0.0;
        result[2][0] = -s;
        result[2][1] = 0.0;
        result[2][2] = c;
        return result;
    }
}

//float[][] matYX(float ax, float ay) {
//    float[][] result = new float[3][3];
//    float sx = sin(ax);
//    float cx = cos(ax);
//    float sy = sin(ay);
//    float cy = cos(ay);
//    result[0][0] = cy;
//    result[0][1] = sx * sy;
//    result[0][2] = cx * sy;
//    result[1][0] = 0.0;
//    result[1][1] = cx;
//    result[1][2] = -sx;
//    result[2][0] = -sy;
//    result[2][1] = sx * cy;
//    result[2][2] = cx * cy;
//    return result;
//}

//void vectorRotate(float[] v, float ax, float ay) {
//    // Matrix(Y) * MAtrix(X)
//    float[] temp = {v[0], v[1], v[2]};
//    float sx = sin(ax);
//    float cx = cos(ax);
//    float sy = sin(ay);
//    float cy = cos(ay);
//    v[0] = cy * temp[0];
//    v[0] += sx * sy * temp[1];
//    v[0] += cx * sy * temp[2];
//    v[1] = 0.0;
//    v[1] += cx * temp[1];
//    v[1] += -sx * temp[2];
//    v[2] = -sy * temp[0];
//    v[2] += sx * cy * temp[1];
//    v[2] += cx * cy * temp[2];
//}